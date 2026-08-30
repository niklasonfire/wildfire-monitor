# Host build. The firmware is built by ./idf.sh in the ESP-IDF container; this
# builds the part of it that does not need a board - the pure decoding in
# main/wfdecode - and replays recorded Captures through it.
#
#   make test        build and replay every fixture (the one command)
#   make fixtures    rebuild the .wfl fixtures from the checked-in dumps
#   make clean
#
# Deliberately plain: gcc and GNU make, nothing to install.

CC       ?= gcc
OPT      ?= -O2
WARN      = -Wall -Wextra -Werror
# The decoding is C99 and nothing else, which is what lets it compile both here
# and in the ESP-IDF build. The harness around it may use POSIX.
PURE_STD  = -std=c99
HOST_STD  = -std=c99 -D_DEFAULT_SOURCE

BUILD     = build-host
WFDECODE  = main/wfdecode
FIXTURES  = tests/fixtures
PYTHON   ?= python3

PURE_OBJ  = $(BUILD)/wfdecode.o $(BUILD)/wfl_read.o
REPLAY    = $(BUILD)/replay

.PHONY: test test-replay test-scripts fixtures clean

test: test-replay test-scripts

test-replay: $(REPLAY)
	$(REPLAY) $(FIXTURES)

test-scripts:
	$(PYTHON) -m unittest discover -s tests -p 'test_*.py' -v

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/%.o: $(WFDECODE)/%.c | $(BUILD)
	$(CC) $(PURE_STD) $(WARN) $(OPT) -I$(WFDECODE) -c $< -o $@

$(REPLAY): tests/host/replay.c $(PURE_OBJ) | $(BUILD)
	$(CC) $(HOST_STD) $(WARN) $(OPT) -I$(WFDECODE) $^ -o $@

# The dumps are the only surviving copy of these rides; the .wfl next to them
# is rebuilt from that text, never edited by hand.
fixtures:
	./scripts/dump2wfl.py captures/cap0007_dump.log $(FIXTURES)/cap0007.wfl

clean:
	rm -rf $(BUILD)
