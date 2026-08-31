---
status: accepted
---

# The Field Table generates every decoder

The meaning of every byte the two devices send is declared once, in a
machine-readable Field Table. The firmware decoder, the offline decoder and the
field documentation are all generated from it at build time. None of the three
is edited by hand.

## Why

ADR-0001 forces decoding to exist in two languages. Hand-maintaining both, plus
prose documentation describing them, is three copies of one truth and they will
disagree.

They already have. The field documentation was hand-transcribed from an
upstream project that only ever implemented one of the Controller's three
live-telemetry blocks. The transcription faithfully inherited that gap, and the
document then read as authoritative — leading us to conclude the Controller
reported no voltage or current at all. It reports both, at 35 Hz, and had been
recording them to flash undecoded for two rides. A generated document could not
have made that claim.

The Field Table also carries per-field Confidence, so the gap between "upstream
says so" and "we proved it against an independent measurement" is visible
everywhere it matters rather than lost in prose.

## Consequences

Fields whose decoding is not a simple offset-and-scale — speed, which spans two
frame types, and phase current, with its 24-bit big-endian encoding and square
root — stay hand-written in both languages. The Field Table covers the simple
majority and does not grow a DSL to cover the rest.

Generation runs during the build, inside the ESP-IDF container.
