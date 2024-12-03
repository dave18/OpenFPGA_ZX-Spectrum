# OpenFPGA_ZX-Spectrum
OpenFPGA Port of the MiSTER Spectrum Core to Analogue Pocket

Original here https://github.com/MiSTer-devel/ZX-Spectrum_MISTer.git

Core requires the file boot.rom to be copied to Assets\zxspectrum\dave18.ZXSpectrum\boot.rom.  This is a consolidated rom file and is not provided. Please see original MiSTer readme for boot.rom structure.

v0.2.0-alpha
Improved pause function (aligns with rising edge of cpu clock to avoid (hopefully) crashes).  Sound is muted when paused.
Sub-directories (Cores/Assets/Platforms) now start with capital letter
Core name changed from 'ZX Spectrum 48K' to 'X Spectrum'

v0.1.0-alpha
All hardware features implemented except disk drives and DivMMC
- Note: I haven't tested General Sound as I haven't yet found a non-TRD file that uses it!

Pocket specfic features
Virtual keyboard (very basic for now)
Key Mapped Joystick (allows Left, Right, Up, Down, A, B, X, Y, L Trig and R Trig to be mapped to keyboard keys).
- Note: if using a different joystick type X, Y, L Trig and R Trig will still function to allow for extra keys to be mapped
Border/Borderless video option
Select brings up virtual keyboard
Start brings up menu
L Trig & Start - Pause/Unpause - warning, this may crash you game as it just hold CPU clock cycles

TODO:
Disk drive
DivMMC

Aspirations:
Dock mode
Save States
Save per game Config

- Note: on my machine (AMD Ryzen 7 5800X) I have to restrict Quartus to a single core (set_global_assignment -name NUM_PARALLEL_PROCESSORS 1) otherwise it crashes with Access Violation errors.  Annonying as it takes longer to build but I'm using v18.1 as per the APF framework so maybe it's a big buggy running on multiple cores.
