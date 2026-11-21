CC ?= gcc
CFLAGS ?= -std=c11 -O2 -Wall -Wextra -Wpedantic
CPPFLAGS += -Iinclude
LDLIBS += -lm

SRC := $(wildcard src/*.c)
HEADERS := $(wildcard include/*.h)

.PHONY: all run clean
all: channel_sim

channel_sim: $(SRC) $(HEADERS)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(SRC) $(LDLIBS) -o $@

test-geodesy:
	$(CC) $(CPPFLAGS) $(CFLAGS) tests/test_geodesy.c src/geodesy.c $(LDLIBS) -o test_geodesy
	./test_geodesy

run: channel_sim
	./channel_sim config/simulation.ini

clean:
	rm -f channel_sim channel_sim.exe test_geodesy test_geodesy.exe
