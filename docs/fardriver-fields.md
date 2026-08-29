# Fardriver frame fields

None of this is our own reverse-engineering — it is transcribed from two
external sources, checked against our own captures where we have a capture to
check it against:

* [`baranator/blackTeaDisp`](https://github.com/baranator/blackTeaDisp),
  `src/main.cpp` - a working ESP32 dashboard for the same bike (a Blacktea
  Motorbikes Wildfire), same Fardriver controller and Daly BMS. Field offsets
  below are read straight out of its notify callbacks.
* [`jackhumbert/fardriver-controllers`](https://github.com/jackhumbert/fardriver-controllers),
  `fardriver.hpp` - documents the frame envelope and CRC independently, and is
  credited by blackTeaDisp as its own source for the envelope.

## Frame envelope, confirmed by a second source

`aa <type> <12-byte payload> <crc_lo> <crc_hi>`, as in `docs/capture.md`.
jackhumbert's version of the same envelope: byte 1 is a 6-bit ID (0..54)
packed with a 2-bit flags field; the flags we always see are `0b10`, which is
exactly why our type byte runs `0x80..0xb6` rather than `0x00..0x36`. His
`fardriver.hpp` builds its CRC table from the same two seed bytes we use as
the init value (`0x3C`, `0x7F` -> little-endian `0x7f3c`). Two independent
implementations agreeing on an unusual init value means this is the
documented vendor protocol, not something bespoke to our unit.

Payload byte numbering below is 0-based from the start of the 12-byte
payload, i.e. payload byte 0 is frame byte 2.

## Decoded fields

Confidence key: **CONFIRMED** - matches our own cap0006 decode, derived
independently of these sources. **SOUND** - read from blackTeaDisp code that
runs against this exact bike. **UNCERTAIN** - blackTeaDisp's own author
flags it as a guess.

| Type | Payload bytes | Field | Value | Confidence |
| --- | --- | --- | --- | --- |
| `0xb0` | byte 0, bits 2-3 | gear | `(byte0 & 0x0c) >> 2` -> 0/1/2 = eco/standard/sport | CONFIRMED |
| `0xb0` | byte 0, bit 4 | sliding_backwards | flag | SOUND |
| `0xb0` | byte 0, bit 5 | motion | flag | SOUND |
| `0xb0` | byte 3, bit 7 | brake_switch | flag | SOUND |
| `0xb0` | bytes 6-7 | cur_rpm | u16 LE | SOUND |
| `0xaf` | byte 4 | wheel_ratio | u8 | SOUND |
| `0xaf` | byte 5 | wheel_radius | u8 | SOUND |
| `0xaf` | byte 6 | avg_speed_kmh | u8 | SOUND |
| `0xaf` | byte 7 | wheel_width | u8 | SOUND |
| `0xaf` | bytes 8-9 | rate_ratio | u16 LE | SOUND |
| `0xb5` | bytes 0-1 | engine_temp | i16 LE | SOUND |
| `0xb3` | byte 10 | controller_temp | i16, but read from a single byte - width is probably wrong | UNCERTAIN |
| `0xb3` | byte 11 | (unknown) | alternates 0/1 every 30-60 s, then back within ~2 s - not temperature-shaped | UNCERTAIN |
| `0x94` | bytes 8-9 | odometer_raw | u16 LE, incremented roughly per 100 m | SOUND |
| `0x8b` | - | (unknown) | unimplemented upstream too | - |

The gear decode is the useful cross-check: our cap0006 capture observed
payload byte 0 taking values 1/5/9 across an eco-standard-sport-standard
cycle. `(1 & 0x0c) >> 2 = 0`, `(5 & 0x0c) >> 2 = 1`, `(9 & 0x0c) >> 2 = 2` -
eco/standard/sport, bit for bit. Byte 1's accompanying 24/25/26 is not
explained by this bitfield and remains open.

### Correction to `docs/capture.md` / `README.md`

Both currently say the only fields that move in a parked capture are payload
bytes 8-9 and 10-11 of the eight live types, idling at -1/-2 and 0/0xffff.
That observation still stands as a description of what was captured, but the
conclusion drawn from it needs correcting: **`cur_rpm` lives at payload bytes
6-7 of type `0xb0`, not 8-9/10-11.** Parked, RPM is 0, which is
byte-identical to the 47 truly static types and so didn't register as "live"
in that capture at all - it isn't that RPM is one of the unexplained moving
fields, it's that a stationary bike can't distinguish "live but currently
zero" from "static". Payload bytes 8-9/10-11 of type `0xb0` remain genuinely
unassigned; they are not `rate_ratio`, `odometer_raw`, or anything else in
this table.

This also explains why `0xaf`, `0xb5`, `0x8b`, `0xb3` and `0x94` never showed
up in the "every 7th type is live" pattern from cap0006: wheel config,
temperature and odometer all change far too slowly for a short parked
capture to tell them apart from the 47 genuinely static types. They're live
telemetry, just not on that timescale.

## Daly BMS fields

Lower confidence than the Fardriver table above: blackTeaDisp's byte offsets
are written against whatever response length its own testing produced, and
our own BMS notifications in `captures/mcu_frames.log` are `len=12`, not
whatever length these offsets assume. Verify against a raw capture before
relying on this table.

Request frame (13 bytes, written to `0xfff2`): `a5 80 <cmd> 08 00 00 00 00 00
00 00 00 <crc>`, single-byte CRC. Commands: `0x90` SOC, `0x91` V-range,
`0x92` T-range, `0x98` errors, `0x50` params.

| Command | Byte(s) | Field | Value |
| --- | --- | --- | --- |
| `0x90` SOC | 4-5 | volt_tot_mv | u16 BE `* 100` |
| `0x90` SOC | 8-9 | current_ma | `(u16 BE - 30000) * 100` |
| `0x90` SOC | 10-11 | soc_perm | u16 BE, per-mille |
| `0x91` V-range | 4-5 | highest_v_mv | u16 BE |
| `0x91` V-range | 6 | highest_v_cell | u8 |
| `0x91` V-range | 7-8 | lowest_v_mv | u16 BE |
| `0x91` V-range | 9 | lowest_v_cell | u8 |
| `0x92` T-range | 4 | highest_temp | `u8 - 40` |
| `0x92` T-range | 5 | highest_temp_cell | u8 |
| `0x92` T-range | 6 | lowest_temp | `u8 - 40` |
| `0x92` T-range | 7 | lowest_temp_cell | u8 |
| `0x50` params | 4-7 | capacity_mah | u32 BE |

## Product direction

Plan for the firmware: a **live** display mode showing the CONFIRMED/SOUND
fields above in real time while riding (gear, speed/rpm, brake, motion,
engine/controller temp, odometer), independent of the existing capture/dump
flow. The M5StickC's buttons should cycle between this live view and the
existing capture views, rather than the two being separate firmware modes.
Not implemented yet - this section is the target for whoever builds the
decoder and display code next.
