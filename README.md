# OpenFPGA_ZX-Spectrum
OpenFPGA Port of the MiSTER Spectrum Core to Analogue Pocket

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

- Note: on my machine (AMD Ryzen 7 5800X) I have to restrict Quartus to a single core (set_global_assignment -name NUM_PARALLEL_PROCESSORS 1) otherwise it crashes with Access Violation errors.  Annonying as it takes longer to build but I'm using v18.1 as per the APF framework so maybe it's a big buggy running on multiple cores).
