CC ?= cc
CFLAGS ?= -O3 -std=c11 -march=native -Wall -Wextra -Wpedantic
LDLIBS ?= -lm

all: metaldrone

metaldrone: metaldrone.c
	$(CC) $(CFLAGS) -o $@ $< $(LDLIBS)

clean:
	rm -f metaldrone

.PHONY: all clean
