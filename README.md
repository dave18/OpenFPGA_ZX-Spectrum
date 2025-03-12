# OpenFPGA_ZX-Spectrum
OpenFPGA Port of the MiSTER Spectrum Core to Analogue Pocket

Original here https://github.com/MiSTer-devel/ZX-Spectrum_MISTer.git

Core requires the file boot.rom to be copied to Assets\zxspectrum\dave18.ZXSpectrum\boot.rom.  This is a consolidated rom file and is not provided. 

### boot.rom Structure

boot.rom is a collection of required ROMs, however it does not contain a full set of ROMs for each supported machine and/or hardware, it only includes the necessary subset.

| N | Base Offset (Hex) | Chunk Size (Hex) | SHA1 Of the Original | Description |
| ---: | ---: | ---: | :---: | :--- | 
|  1 | 00000 | 2000 | B6F3F5F381D67EED24AB214698CB2B3B7D6091DA | ESXDOS 0.8.9 (Can be replaced as long as rom matches VHD version or left as zeros to disable) |
|  2 | 02000 | 2000 | N/A | zeroes.bin unused 8K, padding to fill remaining space in 16K block |
|  3 | 04000 | 4000 | 5f40f5af51c4c1e9083eac095349d7518545b0e0 | glukpen.rom Mr Gluk Boot Service version 6.61 |
|  4 | 08000 | 4000 | ??? (doesn't match known 5.04t or 5.04t-bugfixed) | TR-DOS 5.04T |
|  5 | 0C000 | 4000 | d07fcdeca892ee80494d286ea9ea5bf3928a1aca | 128p-0.rom (128K editor and menu), the first half of the Pentagon 128 ROM |
|  6 | 10000 | 4000 | 80080644289ed93d71a1103992a154cc9802b2fa | 128-1.rom English 128 ROM 1 (48K BASIC) |
|  7 | 14000 | 4000 | 62ec15a4af56cd1d206d0bd7011eac7c889a595d | plus3-4.1-english-0.rom English +2B/+3B v4.1 ROM 0 (128K editor) |
|  8 | 18000 | 4000 | 1a7812c383a3701e90e88d1da086efb0c033ac72 | plus3-4.1-english-1.rom English +2B/+3B v4.1 ROM 1 (128K syntax checker) |
|  9 | 1C000 | 4000 | 8df145d10ff78f98138682ea15ebccb2874bf759 | plus3-4.1-english-2.rom English +2B/+3B v4.1 ROM 2 (+3DOS) |
| 10 | 20000 | 4000 | be365f331942ec7ec35456b641dac56a0dbfe1f0 | plus3-4.1-english-3.rom English +2B/+3B v4.1 ROM 3 (48K BASIC) |
| 11 | 22000 | 2000 | 6b841dc5797ef7eb219ad455cd1e434ca3b9d30d | plusd-1.A.rom MGT's +D disk interface ROM v1.A |
| 12 | 24000 | 2000 | **CUSTOM** | plusd-sys.bin MODIFIED version of the +D system code, based on the chunk CONFIG2 from "Plus D System Tape" |
| 13 | 26000 | 2000 | 8df204ab490b87c389971ce0c7fb5f9cbd281f14 | mf128-87.2.rom (CRC32: 78ec8cfd) Miltuface 128 (87.2) |
| 14 | 28000 | 2000 | 926425b3e84180683f0872aee9ebf6f4b9dfaf5f | genie128-2.rom GENIE 128K V2.1, the second half of the Genie 128 Disassembler ROM |
| 15 | 2A000 | 2000 | 5d74d2e2e5a537639da92ff120f8a6d86f474495 | mf3-3.C.rom (CRC32: 2d594640) Multiface 3 (3.C) |
| 16 | 2C000 | 2000 | N/A | zeroes.bin unused 8K, padding for the MF3 ROM to fill remaining space in 16K block |
| 17 | 30000 | 4000 | 5ea7c2b824672e914525d1d5c419d71b84a426a2 | 48.rom BASIC for 16/48K models |

### Pocket specfic features

Virtual keyboard (very basic for now - A press key, X toggle shifts)

Key Mapped Joystick (allows Left, Right, Up, Down, A, B, X, Y, L Trig and R Trig to be mapped to keyboard keys).

- Note: if using a different joystick type X, Y, L Trig and R Trig will still function to allow for extra keys to be mapped

Border/Borderless video option

Select brings up virtual keyboard

Start brings up menu

L Trig & Start - Pause/Unpause - warning, this may crash you game as it just holds CPU clock cycles

L Trig & Select - Issue NMI



### Change Log

### v0.7.0-beta

Reverted to a version of the MiSTER SDRAM controller but adapted to AGG23's timings to fix compatability with Turbo modes

Added - New Tape menu

Added - Tape Pause/Prev/Next options

Fixed - Tape sound on/off not working

Fixed - Kempston inputs still active while Virtual Keyboard displayed

Fixed - Auto-typing of LOAD ""

### v0.6.0-beta

Changed SDRAM to AGG23's version to fix timings on some Pockets

### v0.5.0-beta

Fixed sound output (signed rather than unsigned output)

Fixed - NMI shortcut using Left Shoulder + Select

Added - Optional disk LED indicator

and more changes to timing constraints to improve compatability across devices

### v0.4.0-beta

Fixed General Sound Hardware

Adjusted timing contraints to (hopefully) fix issues some people were experiencing

### v0.3.0-beta

Added disk drives

Added DivMMC 

Added Multiface 128

Added NMI

Added Quick Model Select

Added additional reset options

Change: Virtual Keyboard allows to hold Shift (X Button toggles)

Change: ROM Structure has changed to incorporate DiVMMC (0x0000 to 0x2000 can be left as zeros if DivMMC emulation not required).


### v0.2.0-alpha

Improved pause function (aligns with rising edge of cpu clock to avoid (hopefully) crashes).  Sound is muted when paused.

Sub-directories (Cores/Assets/Platforms) now start with capital letter

Core name changed from 'ZX Spectrum 48K' to 'ZX Spectrum'


### v0.1.0-alpha

All hardware features implemented except disk drives and DivMMC

- Note: I haven't tested General Sound as I haven't yet found a non-TRD file that uses it!



### TODO
Save States

Save per game Config



### Aspirations

Dock mode


### Note:

On my machine (AMD Ryzen 7 5800X) I have to restrict Quartus to a single core (set_global_assignment -name NUM_PARALLEL_PROCESSORS 1) otherwise it crashes with Access Violation errors.  Annoying as it takes longer to build but I'm using v18.1 as per the APF framework so maybe it's a big buggy running on multiple cores.
