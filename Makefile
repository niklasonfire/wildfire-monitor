# Host build. The firmware is built by ./idf.sh in the ESP-IDF container; this
# builds the parts of it that do not need a board - the pure decoding in
# main/wfdecode and the pure estimation in main/wfest - and replays recorded
# Captures through them.
#
#   make test        generate, build, unit-test and replay every fixture
#   make gen         just run the Field Table through the generator
#   make docs        regenerate docs/field-table.md from the Field Table
#   make fit         refit Consumption against speed over the whole archive
#   make fixtures    rebuild the .wfl fixtures from the checked-in dumps
#   make clean
#
# Deliberately plain: gcc, GNU make and the Python standard library, nothing to
# install.

CC       ?= gcc
OPT      ?= -O2
WARN      = -Wall -Wextra -Werror
# Determinism. GCC contracts a*b+c into a fused multiply-add by default, which
# rounds once instead of twice - a better answer, and a different one depending
# on what the compiler chose to fuse. main/wfest has to produce the same
# Remaining Energy curve here and on the Monitor, so the licence is withdrawn
# in both builds; see main/CMakeLists.txt for the other half.
FPDET     = -ffp-contract=off
# The decoding is C99 and nothing else, which is what lets it compile both here
# and in the ESP-IDF build. The harness around it may use POSIX.
PURE_STD  = -std=c99
HOST_STD  = -std=c99 -D_DEFAULT_SOURCE

BUILD     = build-host
WFDECODE  = main/wfdecode
WFEST     = main/wfest
WFOTA     = main/wfota
FIXTURES  = tests/fixtures
PYTHON   ?= python3

# ADR-0002: the decoders are generated from the Field Table, into the build
# directory, which is gitignored - so nothing can be edited into them and
# survive a rebuild. The document is the one generated artefact that is
# committed, because a document nobody can read on the way past is not
# documentation; `make test` fails if it has drifted from the table.
TABLE     = field-table.json
GENERATOR = scripts/gen_fields.py
DOC       = docs/field-table.md
GEN       = $(BUILD)/gen
GEN_STAMP = $(GEN)/.generated

PURE_OBJ  = $(BUILD)/wfdecode.o $(BUILD)/wfl_read.o $(BUILD)/wf_fields.o \
            $(BUILD)/wfest.o $(BUILD)/wfota.o
INC       = -I$(WFDECODE) -I$(WFEST) -I$(WFOTA) -I$(GEN)
REPLAY    = $(BUILD)/replay
UNIT      = $(BUILD)/unit

FITTER    = scripts/fit_consumption.py
FIT_H     = $(WFEST)/wf_fit.h

.PHONY: test test-unit test-replay test-scripts gen docs fit fixtures clean

test: test-unit test-replay test-scripts

# The decoding a recorded ride cannot reach: the Odometer wrap, which no
# fixture we hold crosses, the power block's offsets read out of a frame built
# byte by byte, and the manifests update mode has to refuse - which no release
# will ever publish. See tests/host/unit.c.
test-unit: $(UNIT)
	$(UNIT)

test-replay: $(REPLAY)
	$(REPLAY) $(FIXTURES)

# Needs the replay binary too: the cross-language test runs it and compares
# what it decodes against what the generated Python decoder decodes.
test-scripts: $(REPLAY) $(GEN_STAMP)
	$(PYTHON) -m unittest discover -s tests -p 'test_*.py' -v

gen: $(GEN_STAMP)

docs:
	$(PYTHON) $(GENERATOR) $(TABLE) --doc $(DOC)

# Issue #19. The samples come out of the real estimator - $(REPLAY) --samples
# walks every Capture through main/wfest and prints differences of its own
# totals - and the Python does nothing but the regression. That split is the
# point: a Python energy-and-distance integrator would be a second answer to
# a question ADR-0004 gives one answer to.
#
# Not part of `make test`, because it writes a committed file. `make test`
# checks instead that the committed file is still what today's archive
# produces, which is the same discipline docs/field-table.md is held to.
fit: $(REPLAY)
	$(PYTHON) $(FITTER) --captures $(FIXTURES) --header $(FIT_H)

$(BUILD):
	mkdir -p $(BUILD)

$(GEN_STAMP): $(TABLE) $(GENERATOR) | $(BUILD)
	$(PYTHON) $(GENERATOR) $(TABLE) --c-dir $(GEN) --py-dir $(GEN) --stamp $@

$(BUILD)/wf_fields.o: $(GEN_STAMP) | $(BUILD)
	$(CC) $(PURE_STD) $(WARN) $(OPT) $(FPDET) $(INC) \
	    -c $(GEN)/wf_fields.c -o $@

# wfest.h includes wf_fit.h, so the fitted coefficients are compiled into this
# object. Naming the header here is what makes `make fit` followed by
# `make test` mean anything: without it the estimator keeps the coefficients it
# was last built with, the replay measures a fit nobody ran, and the drift
# check passes against a stale binary.
$(BUILD)/wfest.o: $(WFEST)/wfest.c $(WFEST)/wfest.h $(FIT_H) $(GEN_STAMP) \
                  | $(BUILD)
	$(CC) $(PURE_STD) $(WARN) $(OPT) $(FPDET) $(INC) -c $< -o $@

# The pure half of update mode - the manifest, its URL, and which network to
# join. Nothing generated goes into it, so unlike the two above it does not
# wait on the Field Table.
$(BUILD)/wfota.o: $(WFOTA)/wfota.c $(WFOTA)/wfota.h | $(BUILD)
	$(CC) $(PURE_STD) $(WARN) $(OPT) $(FPDET) $(INC) -c $< -o $@

$(BUILD)/%.o: $(WFDECODE)/%.c $(GEN_STAMP) | $(BUILD)
	$(CC) $(PURE_STD) $(WARN) $(OPT) $(FPDET) $(INC) -c $< -o $@

$(REPLAY): tests/host/replay.c $(FIT_H) $(PURE_OBJ) | $(BUILD)
	$(CC) $(HOST_STD) $(WARN) $(OPT) $(FPDET) $(INC) \
	    tests/host/replay.c $(PURE_OBJ) -o $@

# Built with PURE_STD, not HOST_STD: it links nothing but main/wfdecode and
# main/wfest, so it holds those seams to the same C99-and-nothing-else rule the
# firmware needs.
$(UNIT): tests/host/unit.c $(FIT_H) $(PURE_OBJ) | $(BUILD)
	$(CC) $(PURE_STD) $(WARN) $(OPT) $(FPDET) $(INC) \
	    tests/host/unit.c $(PURE_OBJ) -o $@

# The dumps are the only surviving copy of these rides; the .wfl next to them
# is rebuilt from that text, never edited by hand.
fixtures:
	./scripts/dump2wfl.py captures/cap0007_dump.log $(FIXTURES)/cap0007.wfl

clean:
	rm -rf $(BUILD)
