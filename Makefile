CC ?= gcc
CFLAGS ?= -std=c11 -O2 -Wall -Wextra -Wpedantic
CPPFLAGS += -Iinclude
LDLIBS += -lm
SRC := $(wildcard src/*.c)
HEADERS := $(wildcard include/*.h)
.PHONY: all run test clean
all: channel_sim
channel_sim: $(SRC) $(HEADERS)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(SRC) $(LDLIBS) -o $@
run: channel_sim
	./channel_sim config/simulation.ini
test: test-geodesy
test-geodesy:
	$(CC) $(CPPFLAGS) $(CFLAGS) tests/test_geodesy.c src/geodesy.c src/math3d.c $(LDLIBS) -o test_geodesy
	./test_geodesy
clean:
	rm -f channel_sim channel_sim.exe test_geodesy test_geodesy.exe
