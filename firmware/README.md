# Setup instructions

## Windows

1. Install git by running the following command in a terminal winow: `winget install Git.Git`
2. [Enable long paths both on the operating system and git level](https://arduino-pico.readthedocs.io/en/latest/platformio.html#important-steps-for-windows-users-before-installing) 
3. Install [Visual Studio Code](https://code.visualstudio.com/)
4. Install [PlatformIO IDE Extension](https://platformio.org/install/ide?install=vscode)
5. Clone the repository e.g. using `git clone https://github.com/atopile/swoop.git`
6. Open the repository in Visual Studio Code e.g. by entering the directory `cd swoop` and running the command `code .`
7. Let PlatformIO install all dependencies and reopen the IDE window when asked to
8. Donwload [Zadig](https://zadig.akeo.ie/) and install the WinUSB driver for the connected "RP2 Boot" device when Swoop is connected over USB
![Zadig window](https://forums.raspberrypi.com/download/file.php?id=55573&sid=6964c0e56687c9a6d2d03afeaecb83e3 "Installing WinUSB using Zadig")
9. Build and upload Swoop firmware using the small arrow button in the bottom right of Visual Studio Code
