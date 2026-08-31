# Captures

Raw evidence behind the Fardriver protocol description in the top-level
README. Kept because it cannot be reproduced byte for byte: the controller
rotates its BLE address between sessions and the payloads depend on the state
of the bike at the time.

Both files were taken on 2026-08-29 with the ignition on and the bike
stationary, against `YuanQuFOC274` on characteristic `0xffec` (value handle
`0x0010`, CCCD `0x0011`).

| File | What it holds |
| --- | --- |
| `mcu_frames.log` | `rec dump` of a 20 s recording, 709 frames, 0 dropped. Every CRC verifies with init `0x7f3c`. This is what the frame format, the 35.5 Hz rate, the 55-type cycle and the eight live types were derived from. |
| `mcu_capture.log` | An earlier `probe 30` run: the full GATT walk, `readall` (device information strings, `system_id`, the `0x2a2a` value `"experimental"`) and `nstat`. 211 frames dropped because the 20 KB buffer holds only ~853. |

Both files open with an eight-second `scan`, and both scan tables are
redacted. A scan lists every BLE device in earshot, which here meant thirty
belonging to other people alongside the bike's two. A snapshot of the
neighbours' radios is not the Controller or the BMS talking, so ADR-0001 never
made it Capture content, and it was stripped out of this repository's history
before the repository was published. The `REDACTED` line in each file says how
many entries went. The bike's own two entries stay, because the BMS advertises
its address and its serial to anyone standing next to the bike.

Parked, the live fields idle at -1/-2 and 0/0xffff, so neither capture is
enough to assign physical meaning to them. That needs a capture taken while
riding:

```bash
./wf.sh 'connect name Yuan' 'discover' 'sub all' --listen 120 --capture captures/ride.log
```
