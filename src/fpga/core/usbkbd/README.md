# USB Keyboard Bridge

Converts the Analogue Pocket openFPGA "docked keyboard" controller-bus report
(`cont3_key` / `cont3_joy` / `cont3_trig`) directly into the ZX Spectrum
keyboard matrix, so a real USB keyboard connected via the Analogue Dock can
drive `core/keyboard.sv`.

## Scope: this only drives the Spectrum keyboard matrix

The docked USB keyboard only ever reaches `core/keyboard.sv` via this
bridge, and `keyboard.sv` only ever drives the Spectrum matrix, F1-F11
(`Fn`), and the mod bits `core_top.sv` reads for the reset/speed/pause
shortcuts documented in the top-level README. It has no path to, and
cannot be extended to reach, any of the following:

- This core's own on-screen menu (Start) or virtual keyboard overlay
  (Select) - both read the attached gamepad directly in
  `src/firmware/main.c`'s `processInput()` (`readJoypad()`, i.e.
  `IO_JOYPAD`/`cont1_key`), entirely independent of this bridge.
- The Analogue Pocket's own system menu (the controller's Home button) -
  where files (`.tap`/`.tzx`/`.dsk`/`.z80`/`.sna`) actually get loaded.
  Confirmed by inspecting `src/firmware/host.h`'s controller bit
  definitions: there is no Home/Guide bit anywhere in what the core
  receives. Analogue's platform firmware intercepts it before it ever
  reaches the FPGA, so there is no signal this bridge - or anything else
  in this core - could watch or forward even in principle.

A controller is required for all of the above, keyboard or no keyboard.

## Architecture: no PS/2 intermediate step

Earlier versions of this bridge vendored a MiSTer-style PS/2 translation
layer from the OpenGateware project's `computer-msx` core
(`platform/pocket/interface/keyboard/`) - `apf2hid` extracting raw USB HID
bytes from the APF report, then `kb_fifo`/`key_arbiter`/`key_mgr` tracking
each of up to 14 HID report "slots" through a per-slot state machine and
emitting a MiSTer-style `ps2_key[10:0]` toggle+strobe event, then
`hid2ps2_key`/`hid2ps2_mod` ROMs translating HID codes to PS/2 codes for
`keyboard.sv`'s PS/2-to-matrix decoder to consume.

That whole shape - one event at a time, tracked per HID slot rather than
per key - was the root cause of essentially every keyboard bug found during
that bridge's development (see git history for the full account): a HID
host reordering/"compacting" its scancode slots when one of several held
keys released could make the vacated slot emit a stale break for a key
that was actually still held elsewhere; fixing that with a "live snapshot"
suppression layer on top introduced a second bug (a poisoned comparison
register that could drop a later, unrelated key); and getting the
toggle/strobe event safely across the clk_74a/clk_sys clock boundary
without losing or merging back-to-back events needed a purpose-built
queue-and-handshake protocol. All of that complexity existed to preserve a
single-event-at-a-time abstraction that a 6-scancode-slot HID report never
actually has.

This version removes it entirely. `apf2hid.sv` (kept, vendored, MIT -
license header below) still extracts the raw modifier byte and 6 scancode
bytes from the APF report and synchronizes them onto `clk_sys`.
`keyboard.sv` decodes that snapshot directly into the Spectrum matrix, from
scratch, every `clk_sys` cycle - see the "USB HID -> Spectrum matrix
decode" section at the top of that file for the mapping tables and the
CAPS SHIFT/SYMBOL SHIFT priority logic. There is no PS/2 code, toggle bit,
strobe pulse, per-slot state, or clock-domain crossing left anywhere in
this path: the whole design is now a single clock domain, and because the
matrix is *re-derived* from the live HID report rather than *incrementally
updated* by discrete events, there is no persistent per-key state that a
slot reorder, a dropped event, or a race between two clocks could ever
leave out of sync. Two keys changing in the same HID report, or a host
reordering its scancode slots, cannot produce a stuck or dropped key -
there's no state left for that class of bug to live in.

`apf2hid.sv`'s license header:

https://github.com/opengateware/computer-msx

Copyright (c) 2023, Marcus Andrade <marcus@opengateware.org>

`SPDX-License-Identifier: MIT` (see the file itself for the full text).

## Local modification: `kb_fifo.sv`'s key-repeat controller (historical)

The vendored typematic-repeat FIFO controller that used to need disabling
here no longer exists in this design at all - it was part of the deleted
`kb_fifo.sv`/`key_arbiter.sv`/`key_mgr.sv` slot-tracking pipeline. No
typematic auto-repeat is implemented (not needed for game controls, the
original priority for keeping it disabled).

## Running the testbenches

Self-contained Icarus Verilog testbenches instantiate `keyboard.sv` exactly
as `core_top.sv` wires it - a single `clk_sys` and the 7 raw HID bytes
(`hid_mod`, `hid_sc1`..`hid_sc6`) as direct inputs, no bridge module, no
second clock:

- `tb_keyboard.sv` - single key, including a long hold
- `tb_multikey.sv` - two/three keys at once, including releasing one while
  others stay held (the old "compaction" scenario - kept as a permanent
  regression guard, though the bug class it originally caught can't recur
  in this design)
- `tb_modifier.sv` - Shift/Ctrl+key combos, both press orders, left and
  right modifiers
- `tb_special.sv` - punctuation keys that force SYMBOL SHIFT and pick a
  shift-dependent target, arrow/backspace/caps-lock/escape keys that force
  CAPS SHIFT, the Tab "G mode" combo, and F-key level detection
- `tb_mash.sv` - all 8 pairs of the QAOP+Space beat-em-up scheme, holding
  one key while rapidly mashing another
- `tb_mash3.sv` - three keys at once (hold two, mash a third)

All six pass. Unlike the old PS/2 bridge, `keyboard.sv` needs no
Icarus-only compatibility patches - it doesn't use any of the SystemVerilog
constructs that used to need working around (no enums, no `always_comb`
with forward-referenced loop variables, no port with both an initializer
and a continuous assign). Run directly:

```
iverilog -g2012 -o tb.vvp tb_keyboard.sv ../keyboard.sv
vvp tb.vvp
```

(substitute any of the other `tb_*.sv` files - none of them need `apf2hid.sv`
either, since they drive `hid_mod`/`hid_sc1..6` directly.)
