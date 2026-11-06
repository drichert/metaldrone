# metaldrone

A dependency-free, offline C program that transforms recorded music into expansive spectral-drone soundscapes. Designed for dense, distorted, rhythmically busy music: hard-rock and 1980s metal become slowly shifting, vaguely recognizable harmonic clouds instead of songs with beats.

## Build

On Linux (GCC or Clang):

```bash
make
# or directly:
cc -O3 -std=c11 -march=native -o metaldrone metaldrone.c -lm
```

No DSP libraries, neural models, GPU, or Internet connection required. FFmpeg is optional and only needed to open compressed audio such as MP3/FLAC. All heavy DSP uses a native radix-2 FFT, lookup-table random phases, and streaming overlap-add output. Processing happens offline and output is written incrementally.

## Run

```bash
./metaldrone metal.wav metal-drone.wav
```

**A few starting points:**

```bash
# Submerged, dark, very slowly evolving
./metaldrone metal.wav abyss.wav --stretch 16 --window 1.8 --pitch -7 --blur 1.2 --cutoff 2300 --wet 0.75 --decay 12

# Recognizable harmonic afterimage; still heavily transformed
./metaldrone metal.wav memory.wav --stretch 6 --window 0.7 --pitch 0 --blur 0.25 --cutoff 5000 --wet 0.4 --decay 5

# Airy, blurred, almost orchestral wash
./metaldrone metal.wav halo.wav --stretch 12 --window 1.4 --pitch 5 --blur 0.8 --cutoff 6800 --wet 0.7 --decay 10
```

The defaults are `--stretch 10 --window 1.2 --pitch -3 --blur 0.55 --wet 0.55 --decay 7 --cutoff 3800 --gain 1 --seed 1`. Use `./metaldrone --help` for parameter ranges.

## Input formats

The executable accepts mono or stereo RIFF/WAVE with 8/16/24/32-bit PCM, or 32/64-bit IEEE float. It also opens MP3, FLAC, and other FFmpeg-supported audio formats **automatically** when the `ffmpeg` program is installed:

```bash
./metaldrone "my metal song.mp3" output.wav
```

For compressed inputs it temporarily decodes to WAV, reads that into memory, and deletes the temporary file before rendering.

Output: **stereo 32-bit float WAV** at the input sample rate. Float WAV avoids quantization noise for successive processing; for regular 16-bit WAV or a listenable compressed file:

```bash
ffmpeg -i output.wav -c:a pcm_s16le output-16bit.wav
ffmpeg -i output.wav -c:a libmp3lame -q:a 2 output.mp3
```

## How it works

1. Move slowly through the original track with very long FFT windows; multiply its duration by `--stretch`.
2. Follow a gradually averaged magnitude spectrum, which suppresses fast rhythmic attacks without erasing the chord and note content entirely.
3. Remap spectral frequencies using `--pitch`, softly remove sub-bass rumble and hiss, and slowly modulate the spectrum in logarithmic frequency bands.
4. Resynthesize with fresh random phase per spectral bin and overlap-add. The original timing and detailed timbres melt into cloudlike sound.
5. Feed the result through an eight-line, orthogonal-feedback stereo reverb for long spatial decay.

A larger `--window` makes the texture more harmonically stable and less event-like. Larger `--blur` makes changes more gradual. `--cutoff` is especially helpful on cymbal-heavy tracks. For faintly recognizable material, keep pitch at zero, blur low, and stretch around 5–8x. For a nearly unrecognizable abyss, use 12–25x, a longer window, and pitch down.

## Limits / practical notes

- A ten-minute input at 10x stretch creates about 100 minutes of audio and a large WAV file. The program checks the standard RIFF 4 GiB size limit before rendering. For long sessions, process shorter sections and join them with `ffmpeg`.
- Source audio is loaded into RAM for fast random access; output audio is streamed in small blocks. Memory grows with the size of the source, not the size of the output.
- It will not preserve the original beat, singer, or riff articulation. It retains harmonic trajectory and spectral fingerprints. This is intentional.
- No multi-threading in this version; run independent songs in parallel if needed.
- For lower CPU cost or smaller files, decrease `--stretch` or resample the input to 32 kHz before processing. No need for a GPU.

## License

MIT. Use and modify freely.
