// ZX Spectrum for Altera DE1
//
// Copyright (c) 2009-2011 Mike Stirling
// Copyright (c) 2015-2017 Sorgelig
//
// All rights reserved
//
// Redistribution and use in source and synthezised forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
//
// * Redistributions in synthesized form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// * Neither the name of the author nor the names of other contributors may
//   be used to endorse or promote products derived from this software without
//   specific prior written agreement from the author.
//
// * License is granted for non-commercial use only.  A fee may not be charged
//   for redistributions as source code or in synthesized/hardware form without
//   specific prior written agreement from the author.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// ----------------------------------------------------------------------------
// Rewritten 2026 to decode a USB HID report directly to the Spectrum matrix,
// with no PS/2 intermediate step - the ghosting-emulation matrix[]/key_data
// logic below is the only part carried over unchanged from the original
// MiSTer PS/2 version of this file. See "USB HID -> Spectrum matrix decode"
// below and src/fpga/core/usbkbd/README.md for why and what changed.
// ----------------------------------------------------------------------------

module keyboard
(
	input             reset,
	input             clk_sys,

	// Currently-held USB HID state, already synchronized into clk_sys as a
	// continuously-refreshed level snapshot (see core_top.sv) - not PS/2,
	// not an edge/toggle event stream. hid_mod is the USB HID modifier byte
	// (bit0=LCtrl,1=LShift,2=LAlt,3=LGUI,4=RCtrl,5=RShift,6=RAlt,7=RGUI);
	// hid_sc1..hid_sc6 are up to 6 simultaneously-held non-modifier USB HID
	// usage codes (8'h00 = empty slot).
	input       [7:0] hid_mod,
	input       [7:0] hid_sc1,
	input       [7:0] hid_sc2,
	input       [7:0] hid_sc3,
	input       [7:0] hid_sc4,
	input       [7:0] hid_sc5,
	input       [7:0] hid_sc6,

	input             recreated_zx,
	input             ghosting,

	input      [15:0] addr,
	output      [4:0] key_data,

	output reg [11:1] Fn = 0,
	output reg  [2:0] mod = 0
);

reg  [4:0] keys[7:0];
wire [4:0] matrix[7:0];
wire [4:0] nghosting = {5{~ghosting}};

// Keyboard ghosting
// Check which rows are connected for all pairs of rows.
// Two rows are connected if they have a key pressed in the same column.
// When two rows are connected, every key pressed in either appears as pressed in both.
// The equation `keys[a][4:0] | keys[b][4:0]` returns whether they are connected:
// if any bit is 0, they are connected; if all bits are 1, they aren't
// (remember keys[][] uses negative logic). If connected, copy the other
// row's keys to this row.
// This would be shorter if we could do it in columns, e.g. matrix[7:0][0]
// because then there would only be 5 columns instead of 8 rows, hence 5x4=20
// lines instead of 8x7=56. Alas, that's not allowed by Quartus.
assign matrix[0][4:0] = keys[0][4:0] & // row 0
		(((keys[1][4:0] | {5{&(keys[0][4:0] | keys[1][4:0])}}) // if rows 0 and 1 connected, add row 1's keys
		& (keys[2][4:0] | {5{&(keys[0][4:0] | keys[2][4:0])}}) // if rows 0 and 2 connected, add row 2's keys
		& (keys[3][4:0] | {5{&(keys[0][4:0] | keys[3][4:0])}}) // ...
		& (keys[4][4:0] | {5{&(keys[0][4:0] | keys[4][4:0])}})
		& (keys[5][4:0] | {5{&(keys[0][4:0] | keys[5][4:0])}})
		& (keys[6][4:0] | {5{&(keys[0][4:0] | keys[6][4:0])}})
		& (keys[7][4:0] | {5{&(keys[0][4:0] | keys[7][4:0])}})
		) | nghosting);
assign matrix[1][4:0] = keys[1][4:0] & // row 1
		(((keys[0][4:0] | {5{&(keys[1][4:0] | keys[0][4:0])}})
		& (keys[2][4:0] | {5{&(keys[1][4:0] | keys[2][4:0])}})
		& (keys[3][4:0] | {5{&(keys[1][4:0] | keys[3][4:0])}})
		& (keys[4][4:0] | {5{&(keys[1][4:0] | keys[4][4:0])}})
		& (keys[5][4:0] | {5{&(keys[1][4:0] | keys[5][4:0])}})
		& (keys[6][4:0] | {5{&(keys[1][4:0] | keys[6][4:0])}})
		& (keys[7][4:0] | {5{&(keys[1][4:0] | keys[7][4:0])}})
		) | nghosting);
assign matrix[2][4:0] = keys[2][4:0] & // ...
		(((keys[0][4:0] | {5{&(keys[2][4:0] | keys[0][4:0])}})
		& (keys[1][4:0] | {5{&(keys[2][4:0] | keys[1][4:0])}})
		& (keys[3][4:0] | {5{&(keys[2][4:0] | keys[3][4:0])}})
		& (keys[4][4:0] | {5{&(keys[2][4:0] | keys[4][4:0])}})
		& (keys[5][4:0] | {5{&(keys[2][4:0] | keys[5][4:0])}})
		& (keys[6][4:0] | {5{&(keys[2][4:0] | keys[6][4:0])}})
		& (keys[7][4:0] | {5{&(keys[2][4:0] | keys[7][4:0])}})
		) | nghosting);
assign matrix[3][4:0] = keys[3][4:0] &
		(((keys[0][4:0] | {5{&(keys[3][4:0] | keys[0][4:0])}})
		& (keys[1][4:0] | {5{&(keys[3][4:0] | keys[1][4:0])}})
		& (keys[2][4:0] | {5{&(keys[3][4:0] | keys[2][4:0])}})
		& (keys[4][4:0] | {5{&(keys[3][4:0] | keys[4][4:0])}})
		& (keys[5][4:0] | {5{&(keys[3][4:0] | keys[5][4:0])}})
		& (keys[6][4:0] | {5{&(keys[3][4:0] | keys[6][4:0])}})
		& (keys[7][4:0] | {5{&(keys[3][4:0] | keys[7][4:0])}})
		) | nghosting);
assign matrix[4][4:0] = keys[4][4:0] &
		(((keys[0][4:0] | {5{&(keys[4][4:0] | keys[0][4:0])}})
		& (keys[1][4:0] | {5{&(keys[4][4:0] | keys[1][4:0])}})
		& (keys[2][4:0] | {5{&(keys[4][4:0] | keys[2][4:0])}})
		& (keys[3][4:0] | {5{&(keys[4][4:0] | keys[3][4:0])}})
		& (keys[5][4:0] | {5{&(keys[4][4:0] | keys[5][4:0])}})
		& (keys[6][4:0] | {5{&(keys[4][4:0] | keys[6][4:0])}})
		& (keys[7][4:0] | {5{&(keys[4][4:0] | keys[7][4:0])}})
		) | nghosting);
assign matrix[5][4:0] = keys[5][4:0] &
		(((keys[0][4:0] | {5{&(keys[5][4:0] | keys[0][4:0])}})
		& (keys[1][4:0] | {5{&(keys[5][4:0] | keys[1][4:0])}})
		& (keys[2][4:0] | {5{&(keys[5][4:0] | keys[2][4:0])}})
		& (keys[3][4:0] | {5{&(keys[5][4:0] | keys[3][4:0])}})
		& (keys[4][4:0] | {5{&(keys[5][4:0] | keys[4][4:0])}})
		& (keys[6][4:0] | {5{&(keys[5][4:0] | keys[6][4:0])}})
		& (keys[7][4:0] | {5{&(keys[5][4:0] | keys[7][4:0])}})
		) | nghosting);
assign matrix[6][4:0] = keys[6][4:0] &
		(((keys[0][4:0] | {5{&(keys[6][4:0] | keys[0][4:0])}})
		& (keys[1][4:0] | {5{&(keys[6][4:0] | keys[1][4:0])}})
		& (keys[2][4:0] | {5{&(keys[6][4:0] | keys[2][4:0])}})
		& (keys[3][4:0] | {5{&(keys[6][4:0] | keys[3][4:0])}})
		& (keys[4][4:0] | {5{&(keys[6][4:0] | keys[4][4:0])}})
		& (keys[5][4:0] | {5{&(keys[6][4:0] | keys[5][4:0])}})
		& (keys[7][4:0] | {5{&(keys[6][4:0] | keys[7][4:0])}})
		) | nghosting);
assign matrix[7][4:0] = keys[7][4:0] &
		(((keys[0][4:0] | {5{&(keys[7][4:0] | keys[0][4:0])}})
		& (keys[1][4:0] | {5{&(keys[7][4:0] | keys[1][4:0])}})
		& (keys[2][4:0] | {5{&(keys[7][4:0] | keys[2][4:0])}})
		& (keys[3][4:0] | {5{&(keys[7][4:0] | keys[3][4:0])}})
		& (keys[4][4:0] | {5{&(keys[7][4:0] | keys[4][4:0])}})
		& (keys[5][4:0] | {5{&(keys[7][4:0] | keys[5][4:0])}})
		& (keys[6][4:0] | {5{&(keys[7][4:0] | keys[6][4:0])}})
		) | nghosting);

// Output addressed row to ULA
assign key_data = (!addr[8]  ? matrix[0] : 5'b11111)
                 &(!addr[9]  ? matrix[1] : 5'b11111)
                 &(!addr[10] ? matrix[2] : 5'b11111)
                 &(!addr[11] ? matrix[3] : 5'b11111)
                 &(!addr[12] ? matrix[4] : 5'b11111)
                 &(!addr[13] ? matrix[5] : 5'b11111)
                 &(!addr[14] ? matrix[6] : 5'b11111)
                 &(!addr[15] ? matrix[7] : 5'b11111);

// ----------------------------------------------------------------------------
// USB HID -> Spectrum matrix decode
//
// keys[7:0][4:0] is indexed [row][col], active LOW (0 = pressed), same
// layout the ghosting logic above expects. Matrix positions are numbered
// 0..39 as row*5+col for the function below; bit 0 is CAPS SHIFT
// (keys[0][0]) and bit 36 is SYMBOL SHIFT (keys[7][1]).
//
// This whole section replaces what used to be an edge-triggered decoder
// fed one MiSTer-style {toggle,pressed,code} PS/2 event at a time, taken
// from a "live snapshot" workaround that itself existed only to route
// around a slot-indexing bug in the vendored PS/2 bridge - see
// src/fpga/core/usbkbd/README.md's history. Going direct from the HID
// report removes the PS/2 shape (and that whole class of bug) entirely:
// there is no per-key state to fall out of sync, because every cycle
// re-derives the complete matrix from whichever of the up to 6 scancode
// slots + 8 modifier bits are CURRENTLY reported, with nothing latched in
// between except the final register keys[][] itself. Two keys changing in
// the same HID report, or a HID host reordering/compacting its scancode
// slots, can no longer produce a stuck or dropped key - there's no slot
// index or toggle bit anywhere in this design for that kind of bug to
// live in.
wire lshift = hid_mod[1];
wire rshift = hid_mod[5];
wire lctrl  = hid_mod[0];
wire rctrl  = hid_mod[4];
wire lalt   = hid_mod[2];
wire ralt   = hid_mod[6];
wire shift_live = lshift | rshift;
wire ctrl_live  = lctrl  | rctrl;
wire alt_live   = lalt   | ralt;

// Per-slot decode: given one currently-active USB HID usage code and the
// live shift state, return which Spectrum matrix position(s) it asserts,
// and whether it unconditionally demands CAPS SHIFT (arrows/backspace/
// caps-lock/escape - Spectrum has no dedicated keys for these, they're
// CAPS SHIFT + a number-row key) or SYMBOL SHIFT (punctuation that only
// exists on the Spectrum via SYMBOL SHIFT + another key), same grouping
// the original PS/2 case statement used, just keyed by USB HID usage ID -
// composed from the vendored hid2ps2_key.sv ROM's HID->PS/2 table (which
// physical key each USB code is) and this file's own long-standing PS/2
// -> Spectrum-matrix mapping (where that key lives on the matrix), with
// no PS/2 codes actually appearing anywhere below. Return format:
// [39:0] = one-hot(s) of matrix positions this code asserts,
// [40]   = force CAPS SHIFT on (and SYMBOL off, unless another active
//          code's force_symbol also fires - resolved by the caller),
// [41]   = force SYMBOL SHIFT on (and CAPS off, same caveat).
// [42]   = additionally assert CAPS SHIFT (Tab/"G mode" only) - ORed on
//          top of whatever [40]/[41] resolve to, not part of that
//          priority group (matches the original PS/2 version, where
//          Tab's own case ran after and independently of the shift-
//          priority case for the same reason).
function automatic [42:0] decode_hid(input [7:0] hid, input shift_in);
	reg [39:0] b;
	reg        fc, fs, tc;
	begin
		b  = 40'h0;
		fc = 1'b0;
		fs = 1'b0;
		tc = 1'b0;
		case (hid)
			// letters
			8'h1D : b[1]  = 1'b1; // Z
			8'h1B : b[2]  = 1'b1; // X
			8'h06 : b[3]  = 1'b1; // C
			8'h19 : b[4]  = 1'b1; // V
			8'h04 : b[5]  = 1'b1; // A
			8'h16 : b[6]  = 1'b1; // S
			8'h07 : b[7]  = 1'b1; // D
			8'h09 : b[8]  = 1'b1; // F
			8'h0A : b[9]  = 1'b1; // G
			8'h14 : b[10] = 1'b1; // Q
			8'h1A : b[11] = 1'b1; // W
			8'h08 : b[12] = 1'b1; // E
			8'h15 : b[13] = 1'b1; // R
			8'h17 : b[14] = 1'b1; // T
			8'h13 : b[25] = 1'b1; // P
			8'h12 : b[26] = 1'b1; // O
			8'h0C : b[27] = 1'b1; // I
			8'h18 : b[28] = 1'b1; // U
			8'h1C : b[29] = 1'b1; // Y
			8'h0F : b[31] = 1'b1; // L
			8'h0E : b[32] = 1'b1; // K
			8'h0D : b[33] = 1'b1; // J
			8'h0B : b[34] = 1'b1; // H
			8'h10 : b[37] = 1'b1; // M
			8'h11 : b[38] = 1'b1; // N
			8'h05 : b[39] = 1'b1; // B

			// number row
			8'h1E : b[15] = 1'b1; // 1
			8'h1F : b[16] = 1'b1; // 2
			8'h20 : b[17] = 1'b1; // 3
			8'h21 : b[18] = 1'b1; // 4
			8'h22 : b[19] = 1'b1; // 5
			8'h23 : b[24] = 1'b1; // 6
			8'h24 : b[23] = 1'b1; // 7
			8'h25 : b[22] = 1'b1; // 8
			8'h26 : b[21] = 1'b1; // 9
			8'h27 : b[20] = 1'b1; // 0

			8'h28 : b[30] = 1'b1; // ENTER
			8'h2C : b[35] = 1'b1; // SPACE

			// Cursor keys, backspace, caps lock, escape - CAPS SHIFT +
			// number row, force CAPS SHIFT on and SYMBOL SHIFT off.
			8'h50 : begin b[19] = 1'b1; fc = 1'b1; end // Left  (CAPS 5)
			8'h51 : begin b[24] = 1'b1; fc = 1'b1; end // Down  (CAPS 6)
			8'h52 : begin b[23] = 1'b1; fc = 1'b1; end // Up    (CAPS 7)
			8'h4F : begin b[22] = 1'b1; fc = 1'b1; end // Right (CAPS 8)
			8'h2A : begin b[20] = 1'b1; fc = 1'b1; end // Backspace (CAPS 0)
			8'h39 : begin b[16] = 1'b1; fc = 1'b1; end // Caps Lock (CAPS 2)
			8'h29 : begin b[35] = 1'b1; fc = 1'b1; end // Escape (CAPS SPACE)

			// Punctuation - force SYMBOL SHIFT on and CAPS SHIFT off;
			// several also pick a different matrix target depending on
			// the live shift state, matching a real PC keyboard's
			// unshifted/shifted pair for that key.
			8'h36 : begin fs = 1'b1; if (shift_in) b[14] = 1'b1; else b[37] = 1'b1; end // , <
			8'h37 : begin fs = 1'b1; if (shift_in) b[13] = 1'b1; else b[38] = 1'b1; end // . >
			8'h38 : begin fs = 1'b1; if (shift_in) b[3]  = 1'b1; else b[4]  = 1'b1; end // / ?
			8'h33 : begin fs = 1'b1; if (shift_in) b[1]  = 1'b1; else b[26] = 1'b1; end // ; :
			8'h34 : begin fs = 1'b1; if (shift_in) b[25] = 1'b1; else b[23] = 1'b1; end // ' "
			8'h2F : begin fs = 1'b1; b[22] = 1'b1; end // [ { give (
			8'h30 : begin fs = 1'b1; b[21] = 1'b1; end // ] } give )
			8'h2D : begin fs = 1'b1; if (shift_in) b[20] = 1'b1; else b[33] = 1'b1; end // - _
			8'h2E : begin fs = 1'b1; if (shift_in) b[32] = 1'b1; else b[31] = 1'b1; end // = +
			8'h35 : begin fs = 1'b1; b[23] = 1'b1; end // ` ~ give '
			8'h55 : begin fs = 1'b1; b[39] = 1'b1; end // keypad * give *
			8'h31 : begin fs = 1'b1; b[39] = 1'b1; end // \ | give *
			8'h32 : begin fs = 1'b1; b[39] = 1'b1; end // Non-US # and ~ give *

			// Tab ("G mode" - EXTEND MODE): asserts CAPS SHIFT and "9"
			// directly, independent of the CAPS/SYMBOL priority group
			// above (does not force SYMBOL off).
			8'h2B : begin tc = 1'b1; b[21] = 1'b1; end

			default: ;
		endcase
		decode_hid = {tc, fs, fc, b};
	end
endfunction

// F1-F11 don't exist on the Spectrum matrix - PC-keyboard-only, used for
// the OSD/menu below. No shift-dependence, no CAPS/SYMBOL interaction.
function automatic [11:1] decode_fn(input [7:0] hid);
	reg [11:1] f;
	begin
		f = 11'h0;
		case (hid)
			8'h3A : f[1]  = 1'b1;
			8'h3B : f[2]  = 1'b1;
			8'h3C : f[3]  = 1'b1;
			8'h3D : f[4]  = 1'b1;
			8'h3E : f[5]  = 1'b1;
			8'h3F : f[6]  = 1'b1;
			8'h40 : f[7]  = 1'b1;
			8'h41 : f[8]  = 1'b1;
			8'h42 : f[9]  = 1'b1;
			8'h43 : f[10] = 1'b1;
			8'h44 : f[11] = 1'b1;
			default: ;
		endcase
		decode_fn = f;
	end
endfunction

wire [42:0] d1 = decode_hid(hid_sc1, shift_live);
wire [42:0] d2 = decode_hid(hid_sc2, shift_live);
wire [42:0] d3 = decode_hid(hid_sc3, shift_live);
wire [42:0] d4 = decode_hid(hid_sc4, shift_live);
wire [42:0] d5 = decode_hid(hid_sc5, shift_live);
wire [42:0] d6 = decode_hid(hid_sc6, shift_live);

wire [39:0] live_bits_raw = d1[39:0] | d2[39:0] | d3[39:0] | d4[39:0] | d5[39:0] | d6[39:0];
wire        any_force_caps   = d1[40] | d2[40] | d3[40] | d4[40] | d5[40] | d6[40];
wire        any_force_symbol = d1[41] | d2[41] | d3[41] | d4[41] | d5[41] | d6[41];
wire        any_tab_caps     = d1[42] | d2[42] | d3[42] | d4[42] | d5[42] | d6[42];

// CAPS/SYMBOL SHIFT baseline continuously follows the physical shift/ctrl
// keys; a currently-active special key (the force_caps/force_symbol
// groups above) overrides that baseline. Both groups being active at
// once (e.g. holding an arrow key while also pressing a punctuation key)
// is not realistic on a physical keyboard - force_symbol wins in that
// case, an arbitrary but deterministic and harmless tie-break. Tab's
// CAPS SHIFT contribution (any_tab_caps) is ORed on afterwards, outside
// this priority resolution - see decode_hid's [42] comment.
wire want_caps   = (any_force_symbol ? 1'b0 : (any_force_caps ? 1'b1 : shift_live)) | any_tab_caps;
wire want_symbol = any_force_symbol ? 1'b1 : (any_force_caps ? 1'b0 : ctrl_live);

wire [39:0] live_bits = (live_bits_raw & ~(40'h1 | (40'h1 << 36)))
                      | (want_caps   ? 40'h1        : 40'h0)
                      | (want_symbol ? (40'h1 << 36) : 40'h0);

wire [11:1] live_fn = decode_fn(hid_sc1) | decode_fn(hid_sc2) | decode_fn(hid_sc3)
                    | decode_fn(hid_sc4) | decode_fn(hid_sc5) | decode_fn(hid_sc6);

// recreated_zx (an alternate, non-QWERTY key mapping) is not currently
// exposed in the Pocket menu - core_top.sv hardwires it to 0 - and its
// original PS/2 implementation was asymmetric make/break (pressing one
// PC key set a target bit low, a DIFFERENT PC key set the same bit high),
// which has no natural equivalent in this continuous, level-based decode.
// Left disconnected here rather than guessing at a redesign for a mode
// nothing can currently reach; re-derive from the PS/2-based history in
// git if this ever needs to be exposed.

// --------------------------------------------------------------------
// "LOAD ""` + ENTER" auto-type macro, triggered by a fresh F10 press.
// Steps through matrix positions on a timer, completely overriding the
// live HID decode while running (matching the original PS/2 version's
// behaviour) - 8'hFF marks "not running"/terminator, 8'hFE marks a gap
// (nothing pressed this step), 0-39 is a direct matrix position index.
// The original PS/2 version's first three running steps defensively
// released Shift/Alt/Ctrl before typing, in case the OSD key combo used
// to reach F10 left one of them latched; that's not a concern here since
// the macro fully overrides live decode rather than incrementally
// patching persistent state, so those steps are now plain gaps.
reg [7:0] auto_seq[46] = '{
	8'hFF,
	8'hFE,8'hFE,8'hFE,8'hFE,8'hFE,8'hFE,8'hFE,8'hFE,
	8'hFE,8'hFE,8'hFE,8'hFE,8'hFE,8'hFE,8'hFE,8'hFE,
	8'hFE,8'hFE,8'hFE,8'hFE,8'hFE,8'hFE,8'hFE,8'hFE,
	8'hFE,8'hFE,8'hFE,8'hFE,8'hFE,8'hFE,8'hFE,8'hFE,
	8'hFE, // was: release right shift
	8'hFE, // was: release alt
	8'hFE, // was: release ctrl
	33,    // press J
	8'hFE, // release J
	23,    // press '
	8'hFE, // release '
	8'hFE, // gap
	23,    // press '
	8'hFE, // release '
	30,    // press ENTER
	8'hFE, // release ENTER
	8'hFF  // terminator
};

reg [5:0] auto_pos = 0;
reg       old_fn10 = 0;

wire        auto_running = (auto_seq[auto_pos] != 8'hFF);
wire [39:0] auto_bits     = (auto_seq[auto_pos] < 40) ? (40'h1 << auto_seq[auto_pos]) : 40'h0;

always @(posedge clk_sys) begin
	reg old_reset = 0;
	integer div;

	old_reset <= reset;
	old_fn10  <= Fn[10];

	Fn  <= live_fn; // F1..F11, level state
	mod <= {lctrl, alt_live, rshift}; // mod[2]=lctrl mod[1]=alt mod[0]=rshift

	if (~old_reset & reset) begin
		keys[0] <= 5'b11111;
		keys[1] <= 5'b11111;
		keys[2] <= 5'b11111;
		keys[3] <= 5'b11111;
		keys[4] <= 5'b11111;
		keys[5] <= 5'b11111;
		keys[6] <= 5'b11111;
		keys[7] <= 5'b11111;
		auto_pos <= 0;
	end else begin
		if (~old_fn10 & Fn[10] & ~auto_running) auto_pos <= 1;

		if (auto_running) begin
			div <= div + 1;
			if (div == 7000000) begin
				div <= 0;
				auto_pos <= auto_pos + 1'd1;
			end
			keys[0] <= ~auto_bits[4:0];
			keys[1] <= ~auto_bits[9:5];
			keys[2] <= ~auto_bits[14:10];
			keys[3] <= ~auto_bits[19:15];
			keys[4] <= ~auto_bits[24:20];
			keys[5] <= ~auto_bits[29:25];
			keys[6] <= ~auto_bits[34:30];
			keys[7] <= ~auto_bits[39:35];
		end else if (~recreated_zx) begin
			div <= 0;
			keys[0] <= ~live_bits[4:0];
			keys[1] <= ~live_bits[9:5];
			keys[2] <= ~live_bits[14:10];
			keys[3] <= ~live_bits[19:15];
			keys[4] <= ~live_bits[24:20];
			keys[5] <= ~live_bits[29:25];
			keys[6] <= ~live_bits[34:30];
			keys[7] <= ~live_bits[39:35];
		end
	end
end

endmodule
