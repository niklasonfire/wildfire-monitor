# Host build. The firmware is built by ./idf.sh in the ESP-IDF container; this
# builds the part of it that does not need a board - the pure decoding in
# main/wfdecode - and replays recorded Captures through it.
#
#   make test        generate, build and replay every fixture (the one command)
#   make gen         just run the Field Table through the generator
#   make docs        regenerate docs/fardriver-fields.md from the Field Table
#   make fixtures    rebuild the .wfl fixtures from the checked-in dumps
#   make clean
#
# Deliberately plain: gcc, GNU make and the Python standard library, nothing to
# install.

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

# ADR-0002: the decoders are generated from the Field Table, into the build
# directory, which is gitignored - so nothing can be edited into them and
# survive a rebuild. The document is the one generated artefact that is
# committed, because a document nobody can read on the way past is not
# documentation; `make test` fails if it has drifted from the table.
TABLE     = field-table.json
GENERATOR = scripts/gen_fields.py
DOC       = docs/fardriver-fields.md
GEN       = $(BUILD)/gen
GEN_STAMP = $(GEN)/.generated

PURE_OBJ  = $(BUILD)/wfdecode.o $(BUILD)/wfl_read.o $(BUILD)/wf_fields.o
REPLAY    = $(BUILD)/replay

.PHONY: test test-replay test-scripts gen docs fixtures clean

test: test-replay test-scripts

test-replay: $(REPLAY)
	$(REPLAY) $(FIXTURES)

# Needs the replay binary too: the cross-language test runs it and compares
# what it decodes against what the generated Python decoder decodes.
test-scripts: $(REPLAY) $(GEN_STAMP)
	$(PYTHON) -m unittest discover -s tests -p 'test_*.py' -v

gen: $(GEN_STAMP)

docs:
	$(PYTHON) $(GENERATOR) $(TABLE) --doc $(DOC)

$(BUILD):
	mkdir -p $(BUILD)

$(GEN_STAMP): $(TABLE) $(GENERATOR) | $(BUILD)
	$(PYTHON) $(GENERATOR) $(TABLE) --c-dir $(GEN) --py-dir $(GEN) --stamp $@

$(BUILD)/wf_fields.o: $(GEN_STAMP) | $(BUILD)
	$(CC) $(PURE_STD) $(WARN) $(OPT) -I$(WFDECODE) -I$(GEN) \
	    -c $(GEN)/wf_fields.c -o $@

$(BUILD)/%.o: $(WFDECODE)/%.c $(GEN_STAMP) | $(BUILD)
	$(CC) $(PURE_STD) $(WARN) $(OPT) -I$(WFDECODE) -I$(GEN) -c $< -o $@

$(REPLAY): tests/host/replay.c $(PURE_OBJ) | $(BUILD)
	$(CC) $(HOST_STD) $(WARN) $(OPT) -I$(WFDECODE) -I$(GEN) \
	    tests/host/replay.c $(PURE_OBJ) -o $@

# The dumps are the only surviving copy of these rides; the .wfl next to them
# is rebuilt from that text, never edited by hand.
fixtures:
	./scripts/dump2wfl.py captures/cap0007_dump.log $(FIXTURES)/cap0007.wfl

clean:
	rm -rf $(BUILD)
