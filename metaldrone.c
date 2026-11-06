/*
 * metaldrone.c - dependency-free C spectral drone / soundscape renderer
 * Copyright (c) 2026. MIT License.
 *
 * Build: cc -O3 -std=c11 -ffast-math -march=native -o metaldrone metaldrone.c -lm
 * Usage: ./metaldrone input.wav output.wav [options]
 *
 * 80s metal -> spectral clouds: slow STFT scanning, magnitude-memory smoothing,
 * random-phase resynthesis, pitch-shifted spectral envelope, slow spectral
 * undulation, and stereo 8-delay feedback-matrix reverb.
 *
 * Input: mono/stereo RIFF/WAVE PCM or float, or MP3/FLAC with FFmpeg installed.
 * Output: stereo IEEE float32 RIFF/WAVE.
 */
#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>
#include <limits.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <errno.h>

#define PI 3.14159265358979323846
#define TAU (2.0 * PI)
#define PHASE_LUT 65536u
#define REVERB_LINES 8
#define BANDS 16
#define OUTBUF_FRAMES 4096

typedef struct { float re, im; } Cpx;
typedef struct {
    uint32_t sample_rate, channels;
    uint64_t frames;
    float *samples; /* interleaved */
    float rms;
} Audio;
typedef struct {
    double stretch, window_seconds, pitch_semitones, blur_seconds;
    double wet, decay_seconds, cutoff_hz, gain;
    uint32_t seed;
} Settings;
typedef struct {
    float *buf[REVERB_LINES];
    int len[REVERB_LINES], at[REVERB_LINES];
    float lp[REVERB_LINES], feedback[REVERB_LINES];
    float wet;
} Reverb;
typedef struct {
    FILE *file;
    uint64_t frames;
    float pending[OUTBUF_FRAMES * 2];
    size_t queued;
} WavOut;

typedef struct {
    size_t n;
    float *tw_re, *tw_im;
} FFTPlan;

typedef struct {
    uint32_t state;
    float *cos_lut, *sin_lut;
} RNG;

static uint16_t rd16(const unsigned char *p) {
    return (uint16_t)((unsigned)p[0] | (unsigned)p[1] << 8);
}
static uint32_t rd32(const unsigned char *p) {
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 |
           (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}
static void wr16(FILE *f, uint16_t x) {
    unsigned char p[2] = {(unsigned char)x, (unsigned char)(x >> 8)};
    fwrite(p, 1, 2, f);
}
static void wr32(FILE *f, uint32_t x) {
    unsigned char p[4] = {(unsigned char)x, (unsigned char)(x >> 8),
                          (unsigned char)(x >> 16), (unsigned char)(x >> 24)};
    fwrite(p, 1, 4, f);
}
static int host_is_le(void) {
    const uint16_t value = 1;
    return *(const unsigned char *)&value == 1;
}
static void *xzalloc(size_t nbytes) {
    void *p = calloc(1, nbytes);
    if (!p) fprintf(stderr, "Out of memory allocating %zu bytes\n", nbytes);
    return p;
}
static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

/* Optional automatic decoding, using FFmpeg only for non-WAV inputs. */
static int input_is_riff_wave(const char *path) {
    unsigned char h[12];
    FILE *f = fopen(path, "rb");
    if (!f) return 0;
    int found = fread(h, 1, sizeof(h), f) == sizeof(h) &&
                !memcmp(h, "RIFF", 4) && !memcmp(h + 8, "WAVE", 4);
    fclose(f);
    return found;
}
static int ffmpeg_decode(const char *path, char *temporary, size_t capacity) {
    const char *pattern = "/tmp/metaldrone-XXXXXX";
    if (strlen(pattern) + 1 > capacity) return 0;
    strcpy(temporary, pattern);
    int fd = mkstemp(temporary);
    if (fd < 0) { perror("make temporary WAV"); return 0; }
    close(fd);
    pid_t pid = fork();
    if (pid < 0) { perror("fork FFmpeg"); unlink(temporary); return 0; }
    if (pid == 0) {
        char *const args[] = {
            "ffmpeg", "-nostdin", "-hide_banner", "-loglevel", "error",
            "-y", "-i", (char *)path, "-vn", "-ac", "2",
            "-c:a", "pcm_f32le", "-f", "wav", temporary, NULL
        };
        execvp("ffmpeg", args);
        perror("exec ffmpeg (install ffmpeg for MP3/FLAC inputs)");
        _exit(127);
    }
    int status = 0;
    pid_t result;
    do { result = waitpid(pid, &status, 0); } while (result < 0 && errno == EINTR);
    if (result < 0 || !WIFEXITED(status) || WEXITSTATUS(status) != 0) {
        fprintf(stderr, "FFmpeg could not decode input: %s\n", path);
        unlink(temporary); return 0;
    }
    return 1;
}

static int audio_read_wav(const char *path, Audio *a) {
    FILE *f = fopen(path, "rb");
    if (!f) { perror(path); return 0; }
    unsigned char head[12];
    if (fread(head, 1, 12, f) != 12 || memcmp(head, "RIFF", 4) ||
        memcmp(head + 8, "WAVE", 4)) {
        fprintf(stderr, "Input must be a RIFF/WAVE file (see README for MP3/FLAC).\n");
        fclose(f); return 0;
    }
    int got_fmt = 0, got_data = 0;
    uint16_t format = 0, channels = 0, bits = 0, block_align = 0;
    uint32_t sr = 0, data_bytes = 0;
    off_t data_off = 0;
    for (;;) {
        unsigned char chunk[8];
        if (fread(chunk, 1, 8, f) != 8) break;
        uint32_t size = rd32(chunk + 4);
        off_t body = ftello(f);
        if (body < 0) break;
        if (!memcmp(chunk, "fmt ", 4)) {
            unsigned char fmt[40] = {0};
            if (size < 16 || fread(fmt, 1, size < 40 ? size : 40, f) < (size < 40 ? size : 40)) break;
            format = rd16(fmt);
            channels = rd16(fmt + 2);
            sr = rd32(fmt + 4);
            block_align = rd16(fmt + 12);
            bits = rd16(fmt + 14);
            if (format == 0xfffe && size >= 40) format = rd16(fmt + 24);
            got_fmt = 1;
        } else if (!memcmp(chunk, "data", 4)) {
            data_off = body;
            data_bytes = size;
            got_data = 1;
        }
        if (fseeko(f, body + (off_t)size + (size & 1u), SEEK_SET) != 0) break;
    }
    if (!got_fmt || !got_data || (channels != 1 && channels != 2) ||
        sr < 8000 || sr > 192000 || (format != 1 && format != 3) ||
        !(bits == 8 || bits == 16 || bits == 24 || bits == 32 || (bits == 64 && format == 3)) ||
        (format == 3 && bits != 32 && bits != 64) ||
        block_align != channels * (bits / 8)) {
        fprintf(stderr, "Unsupported WAV format. Use mono/stereo PCM or float WAV.\n");
        fclose(f); return 0;
    }
    uint64_t frames = data_bytes / block_align;
    if (!frames || frames > SIZE_MAX / (sizeof(float) * channels)) {
        fprintf(stderr, "Input size is invalid or too large.\n"); fclose(f); return 0;
    }
    float *samples = xzalloc((size_t)frames * channels * sizeof(float));
    if (!samples) { fclose(f); return 0; }
    if (fseeko(f, data_off, SEEK_SET) != 0) {
        perror("seek input"); free(samples); fclose(f); return 0;
    }
    unsigned char buf[4096 * 8 * 2];
    const size_t bytes_per = bits / 8;
    uint64_t off = 0;
    double energy = 0.0;
    while (off < frames) {
        size_t count = (size_t)((frames - off) > 4096 ? 4096 : (frames - off));
        size_t need = count * block_align;
        if (fread(buf, 1, need, f) != need) {
            fprintf(stderr, "Unexpected end of WAV audio.\n");
            free(samples); fclose(f); return 0;
        }
        for (size_t j = 0; j < count * channels; ++j) {
            const unsigned char *p = buf + j * bytes_per;
            float v = 0;
            if (format == 3 && bits == 32) {
                memcpy(&v, p, 4);
            } else if (format == 3 && bits == 64) {
                double d;
                memcpy(&d, p, 8);
                v = (float)d;
            } else if (bits == 8) {
                v = ((int)p[0] - 128) / 128.0f;
            } else if (bits == 16) {
                v = (float)(int16_t)rd16(p) / 32768.0f;
            } else if (bits == 24) {
                int32_t x = (int32_t)((uint32_t)p[0] | (uint32_t)p[1] << 8 |
                                      (uint32_t)p[2] << 16);
                if (x & 0x800000) x |= (int32_t)0xff000000u;
                v = x / 8388608.0f;
            } else if (bits == 32) {
                v = (float)(int32_t)rd32(p) / 2147483648.0f;
            }
            if (!isfinite(v)) v = 0;
            samples[(size_t)off * channels + j] = v;
            energy += (double)v * v;
        }
        off += count;
    }
    fclose(f);
    a->sample_rate = sr;
    a->channels = channels;
    a->frames = frames;
    a->samples = samples;
    a->rms = (float)sqrt(energy / ((double)frames * channels));
    return 1;
}

static int wav_out_open(WavOut *w, const char *path, uint32_t sr) {
    memset(w, 0, sizeof(*w));
    w->file = fopen(path, "wb");
    if (!w->file) { perror(path); return 0; }
    /* IEEE float32 stereo WAV, streaming writes; header patched on close. */
    fwrite("RIFF", 1, 4, w->file); wr32(w->file, 0);
    fwrite("WAVEfmt ", 1, 8, w->file); wr32(w->file, 16);
    wr16(w->file, 3); wr16(w->file, 2);
    wr32(w->file, sr); wr32(w->file, sr * 8);
    wr16(w->file, 8); wr16(w->file, 32);
    fwrite("data", 1, 4, w->file); wr32(w->file, 0);
    return !ferror(w->file);
}
static int wav_out_flush(WavOut *w) {
    if (!w->queued) return 1;
    if (fwrite(w->pending, sizeof(float) * 2, w->queued, w->file) != w->queued) {
        perror("write output"); return 0;
    }
    w->frames += w->queued;
    w->queued = 0;
    return 1;
}
static int wav_out_add(WavOut *w, float l, float r) {
    w->pending[2 * w->queued] = l;
    w->pending[2 * w->queued + 1] = r;
    if (++w->queued == OUTBUF_FRAMES) return wav_out_flush(w);
    return 1;
}
static int wav_out_close(WavOut *w) {
    int ok = wav_out_flush(w);
    uint64_t bytes = w->frames * 8;
    if (bytes > UINT32_MAX - 36u) {
        fprintf(stderr, "Output exceeds 4 GiB RIFF/WAV limit. Reduce stretch or split the input.\n");
        ok = 0;
    } else {
        if (fseeko(w->file, 4, SEEK_SET)) ok = 0;
        else wr32(w->file, (uint32_t)(bytes + 36));
        if (fseeko(w->file, 40, SEEK_SET)) ok = 0;
        else wr32(w->file, (uint32_t)bytes);
    }
    if (fclose(w->file) != 0) ok = 0;
    w->file = NULL;
    return ok;
}

static int fft_plan_init(FFTPlan *p, size_t n) {
    p->n = n;
    p->tw_re = xzalloc((n / 2) * sizeof(float));
    p->tw_im = xzalloc((n / 2) * sizeof(float));
    if (!p->tw_re || !p->tw_im) return 0;
    for (size_t i = 0; i < n / 2; ++i) {
        float ang = (float)(-TAU * (double)i / n);
        p->tw_re[i] = cosf(ang);
        p->tw_im[i] = sinf(ang);
    }
    return 1;
}
static void fft_plan_free(FFTPlan *p) {
    free(p->tw_re); free(p->tw_im);
}
static void fft_run(const FFTPlan *p, Cpx *a, int inverse) {
    const size_t n = p->n;
    for (size_t i = 1, j = 0; i < n; ++i) {
        size_t bit = n >> 1;
        while (j & bit) { j ^= bit; bit >>= 1; }
        j ^= bit;
        if (i < j) { Cpx t = a[i]; a[i] = a[j]; a[j] = t; }
    }
    for (size_t len = 2; len <= n; len <<= 1) {
        size_t stride = n / len, half = len >> 1;
        for (size_t i = 0; i < n; i += len) {
            for (size_t j = 0; j < half; ++j) {
                float c = p->tw_re[j * stride];
                float s = inverse ? -p->tw_im[j * stride] : p->tw_im[j * stride];
                Cpx u = a[i + j], v = a[i + j + half];
                float tr = c * v.re - s * v.im;
                float ti = c * v.im + s * v.re;
                a[i + j].re = u.re + tr;
                a[i + j].im = u.im + ti;
                a[i + j + half].re = u.re - tr;
                a[i + j + half].im = u.im - ti;
            }
        }
    }
    if (inverse) {
        const float scale = 1.0f / n;
        for (size_t i = 0; i < n; ++i) {
            a[i].re *= scale;
            a[i].im *= scale;
        }
    }
}
static int rng_init(RNG *r, uint32_t seed) {
    r->state = seed ? seed : 1u;
    r->cos_lut = xzalloc(PHASE_LUT * sizeof(float));
    r->sin_lut = xzalloc(PHASE_LUT * sizeof(float));
    if (!r->cos_lut || !r->sin_lut) return 0;
    for (uint32_t i = 0; i < PHASE_LUT; ++i) {
        float phi = (float)(TAU * i / PHASE_LUT);
        r->cos_lut[i] = cosf(phi);
        r->sin_lut[i] = sinf(phi);
    }
    return 1;
}
static void rng_free(RNG *r) { free(r->cos_lut); free(r->sin_lut); }
static inline uint32_t rand32(RNG *r) {
    uint32_t s = r->state;
    s ^= s << 13; s ^= s >> 17; s ^= s << 5;
    r->state = s;
    return s;
}

static int reverb_init(Reverb *v, int sr, float wet, double decay) {
    static const double delay_s[REVERB_LINES] = {
        0.0637, 0.0719, 0.0797, 0.0871, 0.0947, 0.1051, 0.1153, 0.1277
    };
    memset(v, 0, sizeof(*v));
    v->wet = wet;
    for (int i = 0; i < REVERB_LINES; ++i) {
        v->len[i] = (int)(delay_s[i] * sr) + 1;
        v->buf[i] = xzalloc((size_t)v->len[i] * sizeof(float));
        if (!v->buf[i]) return 0;
        v->feedback[i] = (float)pow(0.001, delay_s[i] / decay);
    }
    return 1;
}
static void reverb_free(Reverb *v) {
    for (int i = 0; i < REVERB_LINES; ++i) free(v->buf[i]);
}
/* 8x8 orthogonal Walsh/Hadamard feedback gives a diffuse, stable reverb. */
static inline void reverb_tick(Reverb *v, float *l, float *r) {
    float y[8], z[8];
    for (int i = 0; i < 8; ++i) {
        float tap = v->buf[i][v->at[i]];
        v->lp[i] = 0.57f * v->lp[i] + 0.43f * tap;
        y[i] = v->lp[i];
    }
    /* Unnormalized Hadamard transform, 3 butterfly stages. */
    float x0 = y[0] + y[1], x1 = y[0] - y[1];
    float x2 = y[2] + y[3], x3 = y[2] - y[3];
    float x4 = y[4] + y[5], x5 = y[4] - y[5];
    float x6 = y[6] + y[7], x7 = y[6] - y[7];
    float a0 = x0 + x2, a1 = x1 + x3, a2 = x0 - x2, a3 = x1 - x3;
    float a4 = x4 + x6, a5 = x5 + x7, a6 = x4 - x6, a7 = x5 - x7;
    const float norm = 0.3535533905932738f;
    z[0] = (a0 + a4) * norm; z[1] = (a1 + a5) * norm;
    z[2] = (a2 + a6) * norm; z[3] = (a3 + a7) * norm;
    z[4] = (a0 - a4) * norm; z[5] = (a1 - a5) * norm;
    z[6] = (a2 - a6) * norm; z[7] = (a3 - a7) * norm;
    float in_l = *l, in_r = *r;
    for (int i = 0; i < 8; ++i) {
        /* Slightly different left/right injection polarity decorrelates stereo. */
        float inj = (i & 1 ? in_r : in_l) * 0.20f;
        v->buf[i][v->at[i]] = inj + v->feedback[i] * z[i];
        if (++v->at[i] == v->len[i]) v->at[i] = 0;
    }
    float out_l = (y[0] + y[1] + y[2] + y[3] - y[4] - y[5] - y[6] - y[7]) * norm;
    float out_r = (y[0] - y[1] + y[2] - y[3] + y[4] - y[5] + y[6] - y[7]) * norm;
    float dry = 1.0f - v->wet * 0.35f;
    *l = dry * in_l + v->wet * out_l;
    *r = dry * in_r + v->wet * out_r;
}

static void help(const char *argv0) {
    fprintf(stderr,
        "metaldrone - turn a song into a long, textured spectral drone\n\n"
        "Usage: %s INPUT.wav OUTPUT.wav [options]\n\n"
        "  --stretch X   Duration multiplier (default 10, range 1..100)\n"
        "  --window S   Spectral window seconds (default 1.2, range .1..4)\n"
        "  --pitch N    Pitch shift in semitones (default -3, range -24..12)\n"
        "  --blur S     Magnitude memory in input seconds (default .55, 0..10)\n"
        "  --wet X      Reverb mix (default .55, 0..1)\n"
        "  --decay S    Reverb decay seconds (default 7, 0.3..30)\n"
        "  --cutoff HZ  High frequency rolloff (default 3800, 300..16000)\n"
        "  --gain X     Output gain multiplier (default 1, 0.05..8)\n"
        "  --seed N     Random texture seed (default 1)\n"
        "  --help       Show this message\n\n"
        "Output: stereo 32-bit float WAV. MP3/FLAC accepted when ffmpeg is installed.\n",
        argv0);
}
static int parse_args(int argc, char **argv, Settings *s) {
    for (int i = 3; i < argc; ++i) {
        if (i + 1 >= argc) return 0;
        const char *key = argv[i++];
        char *end = NULL;
        double val = strtod(argv[i], &end);
        if (!end || *end || !isfinite(val)) return 0;
        if (!strcmp(key, "--stretch")) s->stretch = val;
        else if (!strcmp(key, "--window")) s->window_seconds = val;
        else if (!strcmp(key, "--pitch")) s->pitch_semitones = val;
        else if (!strcmp(key, "--blur")) s->blur_seconds = val;
        else if (!strcmp(key, "--wet")) s->wet = val;
        else if (!strcmp(key, "--decay")) s->decay_seconds = val;
        else if (!strcmp(key, "--cutoff")) s->cutoff_hz = val;
        else if (!strcmp(key, "--gain")) s->gain = val;
        else if (!strcmp(key, "--seed") && val >= 0 && val <= UINT32_MAX && floor(val) == val) s->seed = (uint32_t)val;
        else return 0;
    }
    return s->stretch >= 1 && s->stretch <= 100 &&
           s->window_seconds >= .1 && s->window_seconds <= 4 &&
           s->pitch_semitones >= -24 && s->pitch_semitones <= 12 &&
           s->blur_seconds >= 0 && s->blur_seconds <= 10 &&
           s->wet >= 0 && s->wet <= 1 &&
           s->decay_seconds >= .3 && s->decay_seconds <= 30 &&
           s->cutoff_hz >= 300 && s->cutoff_hz <= 16000 &&
           s->gain >= .05 && s->gain <= 8;
}
static size_t next_power_two(size_t x) {
    size_t n = 1;
    while (n < x) n <<= 1;
    return n;
}
static inline float safe_soft_limit(float x) {
    /* Linear to 0.9, smooth saturation above: no global tanh per sample. */
    float ab = fabsf(x);
    if (ab <= 0.9f) return x;
    float d = ab - 0.9f;
    float y = 0.9f + 0.099f * (d / (0.099f + d));
    return copysignf(y, x);
}

static int render(const Audio *in, const char *output, Settings s) {
    int ok = 0;
    size_t n = next_power_two((size_t)ceil(s.window_seconds * in->sample_rate));
    if (n < 4096) n = 4096;
    if (n > 262144) n = 262144;
    const size_t half = n / 2, hop = n / 4;
    const float ratio = (float)pow(2.0, s.pitch_semitones / 12.0);
    const double hop_in = hop / s.stretch;
    const uint64_t synth_frames = (uint64_t)ceil((double)in->frames * s.stretch / hop);
    const uint64_t tail_samples = s.wet > 0 ? (uint64_t)(in->sample_rate * fmin(20.0, s.decay_seconds * 1.2)) : 0;
    const uint64_t expected_frames = synth_frames * hop + n + tail_samples;
    if (expected_frames > (UINT32_MAX - 36u) / 8u) {
        fprintf(stderr, "Estimated output %.1f GiB exceeds standard WAV 4 GiB limit.\n"
                        "Try a shorter track, shorter stretch, or split the input.\n",
                expected_frames * 8.0 / (1024.0 * 1024 * 1024));
        return 0;
    }
    fprintf(stderr, "Input: %.1fs, %u Hz, %u channel(s)\n", in->frames / (double)in->sample_rate,
            in->sample_rate, in->channels);
    fprintf(stderr, "Render: %.1fx stretch, %zu-point FFT, %.1f semitones, %.2fs blur, %.1fs reverb\n",
            s.stretch, n, s.pitch_semitones, s.blur_seconds, s.decay_seconds);
    fprintf(stderr, "Estimated output: %.1f minutes, ~%.1f MiB\n",
            expected_frames / (60.0 * in->sample_rate), expected_frames * 8.0 / (1024.0 * 1024.0));

    FFTPlan plan = {0}; RNG rng = {0}; Reverb rev = {0}; WavOut out = {0};
    float *window = NULL, *tone = NULL, *smooth[2] = {NULL,NULL};
    float *ring = NULL;
    uint8_t *band_index = NULL;
    Cpx *fftbuf = NULL;
    int out_open = 0;
    if (!fft_plan_init(&plan, n) || !rng_init(&rng, s.seed) ||
        !reverb_init(&rev, (int)in->sample_rate, (float)s.wet, s.decay_seconds)) goto cleanup;
    window = xzalloc(n * sizeof(float));
    tone = xzalloc((half + 1) * sizeof(float));
    band_index = xzalloc((half + 1) * sizeof(uint8_t));
    smooth[0] = xzalloc((half + 1) * sizeof(float));
    smooth[1] = xzalloc((half + 1) * sizeof(float));
    fftbuf = xzalloc(n * sizeof(Cpx));
    /* Power-of-two OLA ring: no allocation or modulo division per sample. */
    const size_t ring_size = next_power_two(n + hop);
    const size_t mask = ring_size - 1;
    ring = xzalloc(ring_size * 2 * sizeof(float));
    if (!window || !tone || !band_index || !smooth[0] || !smooth[1] || !fftbuf || !ring) goto cleanup;
    for (size_t j = 0; j < n; ++j) {
        window[j] = (float)(0.5 - 0.5 * cos(TAU * (double)j / n));
    }
    for (size_t k = 0; k <= half; ++k) {
        double freq = (double)k * in->sample_rate / n;
        double hp = (freq * freq) / (freq * freq + 38.0 * 38.0);
        double f = freq / s.cutoff_hz;
        double lp = 1.0 / sqrt(1.0 + f * f * f * f);
        tone[k] = (float)(hp * lp);
        double band = log2(1.0 + freq / 35.0) * 2.0;
        int b = (int)band;
        band_index[k] = (uint8_t)(b < 0 ? 0 : (b >= BANDS ? BANDS - 1 : b));
    }
    if (!wav_out_open(&out, output, in->sample_rate)) {
        if (out.file) { fclose(out.file); remove(output); out.file = NULL; }
        goto cleanup;
    }
    out_open = 1;

    /* Normalizing the input very gently keeps tracks of varying loudness usable. */
    float source_gain = in->rms > 0.000001f ?
        clampf(0.17f / in->rms, 0.2f, 3.0f) : 1.0f;
    /* Random-phase STFT + Hann overlap otherwise tends to render quietly. */
    source_gain *= (float)s.gain * 2.3f;
    float alpha = s.blur_seconds > 0 ?
        (float)(1.0 - exp(-hop_in / (in->sample_rate * s.blur_seconds))) : 1.0f;
    int initialized[2] = {0,0};
    size_t ring_base = 0;
    float texture_band[BANDS];
    const float mag_scale = 1.0f / sqrtf(ratio);
    uint64_t done = 0;
    uint64_t total = synth_frames + n / hop;
    for (uint64_t frame = 0; frame < total; ++frame) {
        if (frame < synth_frames) {
            double t = (double)frame * hop / in->sample_rate;
            double center_d = (double)frame * hop_in;
            int64_t center = (int64_t)llround(center_d);
            for (int b = 0; b < BANDS; ++b) {
                /* Slow independent movement across logarithmic spectral bands. */
                double phase = TAU * (0.007 + 0.0029 * b) * t + 0.78 * b;
                texture_band[b] = (float)(1.0 + 0.18 * sin(phase));
            }
            for (int ch = 0; ch < 2; ++ch) {
                int read_ch = in->channels == 1 ? 0 : ch;
                for (size_t j = 0; j < n; ++j) {
                    int64_t pos = center + (int64_t)j - (int64_t)half;
                    float sample = 0;
                    if (pos >= 0 && (uint64_t)pos < in->frames)
                        sample = in->samples[(size_t)pos * in->channels + read_ch];
                    fftbuf[j].re = sample * window[j] * source_gain;
                    fftbuf[j].im = 0;
                }
                fft_run(&plan, fftbuf, 0);
                float *sm = smooth[ch];
                for (size_t k = 0; k <= half; ++k) {
                    float re = fftbuf[k].re, im = fftbuf[k].im;
                    float mag = sqrtf(re * re + im * im);
                    sm[k] = initialized[ch] ? sm[k] + alpha * (mag - sm[k]) : mag;
                }
                initialized[ch] = 1;
                /* Real spectral resampling for pitch-shifting without a separate resampler. */
                fftbuf[0].re = fftbuf[0].im = 0;
                for (size_t k = 1; k < half; ++k) {
                    float src = k / ratio;
                    float m = 0;
                    if (src < half) {
                        size_t k0 = (size_t)src;
                        float frac = src - k0;
                        m = (sm[k0] + frac * (sm[k0+1] - sm[k0])) * mag_scale;
                    }
                    m *= tone[k] * texture_band[band_index[k]];
                    uint32_t ri = rand32(&rng) & (PHASE_LUT - 1u);
                    float re = m * rng.cos_lut[ri];
                    float im = m * rng.sin_lut[ri];
                    fftbuf[k].re = re; fftbuf[k].im = im;
                    fftbuf[n-k].re = re; fftbuf[n-k].im = -im;
                }
                fftbuf[half].re = fftbuf[half].im = 0;
                fft_run(&plan, fftbuf, 1);
                for (size_t j = 0; j < n; ++j) {
                    size_t at = ((ring_base + j) & mask) * 2 + ch;
                    ring[at] += fftbuf[j].re * window[j];
                }
            }
        }
        for (size_t j = 0; j < hop; ++j) {
            size_t at = ((ring_base + j) & mask) * 2;
            float l = ring[at], r = ring[at+1];
            ring[at] = ring[at+1] = 0;
            if (s.wet > 0) reverb_tick(&rev, &l, &r);
            if (!wav_out_add(&out, safe_soft_limit(l), safe_soft_limit(r))) goto cleanup;
            ++done;
        }
        ring_base = (ring_base + hop) & mask;
        if (frame % 128 == 0 || frame + 1 == total) {
            fprintf(stderr, "\rRendering: %5.1f%%", 100.0 * (frame + 1) / total);
            fflush(stderr);
        }
    }
    if (s.wet > 0) {
        /* Continue the natural reverb ringdown after the last spectral frame. */
        for (uint64_t j = 0; j < tail_samples; ++j) {
            float l = 0, r = 0;
            reverb_tick(&rev, &l, &r);
            if (!wav_out_add(&out, safe_soft_limit(l), safe_soft_limit(r))) goto cleanup;
            ++done;
        }
    }
    fprintf(stderr, "\n");
    if (!wav_out_close(&out)) { out_open = 0; goto cleanup; }
    out_open = 0;
    fprintf(stderr, "Done: %.1f seconds of stereo float WAV.\n", done / (double)in->sample_rate);
    ok = 1;
cleanup:
    if (out_open) {
        if (out.file) fclose(out.file);
        remove(output);  /* don't leave a corrupted partial file */
    }
    fft_plan_free(&plan); rng_free(&rng); reverb_free(&rev);
    free(window); free(tone); free(band_index); free(smooth[0]); free(smooth[1]);
    free(fftbuf); free(ring);
    return ok;
}
int main(int argc, char **argv) {
    if (!host_is_le()) {
        fprintf(stderr, "This version requires a little-endian machine.\n"); return 1;
    }
    if (argc < 3 || (argc > 1 && !strcmp(argv[1], "--help"))) {
        help(argv[0]); return (argc > 1 && !strcmp(argv[1], "--help")) ? 0 : 1;
    }
    if (!strcmp(argv[1], argv[2])) {
        fprintf(stderr, "Input and output paths must differ.\n"); return 1;
    }
    Settings s = {10, 1.2, -3, 0.55, 0.55, 7, 3800, 1, 1};
    if (!parse_args(argc, argv, &s)) { help(argv[0]); return 1; }
    Audio audio = {0};
    char temporary[64] = {0};
    const char *source = argv[1];
    if (!input_is_riff_wave(source)) {
        FILE *check = fopen(source, "rb");
        if (!check) { perror(source); return 1; }
        fclose(check);
        fprintf(stderr, "Decoding input to temporary WAV with ffmpeg...\n");
        if (!ffmpeg_decode(source, temporary, sizeof(temporary))) return 1;
        source = temporary;
    }
    int read_ok = audio_read_wav(source, &audio);
    if (temporary[0]) unlink(temporary);
    if (!read_ok) return 1;
    int ok = render(&audio, argv[2], s);
    free(audio.samples);
    return ok ? 0 : 1;
}
