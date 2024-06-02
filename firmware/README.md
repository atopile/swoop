# Setup instructions

## Windows

1. Install git by running the following command in a terminal winow: `winget install Git.Git`
1. [Enable long paths both on the operating system and git level](https://arduino-pico.readthedocs.io/en/latest/platformio.html#important-steps-for-windows-users-before-installing)
1. Clone the repository e.g. using `git clone https://github.com/atopile/swoop.git`
1. Install [Visual Studio Code](https://code.visualstudio.com/)
1. Install [PlatformIO IDE Extension](https://platformio.org/install/ide?install=vscode)
1. Reopen Visual Studio Code and open the folder of the repository, it will install all dependencies
1. Donwload [Zadig](https://zadig.akeo.ie/) and install the WinUSB driver for the connected "RP2 Boot" device when Swoop is connected over USB
![Zadig window](https://forums.raspberrypi.com/download/file.php?id=55573&sid=6964c0e56687c9a6d2d03afeaecb83e3 "Installing WinUSB using Zadig")
1. At the bottom left of the window you find buttons for building, uploading, serial telemetry

## Mac

1. Install git e.g. via [Homebrew](https://brew.sh/) using `brew install git` ([formula](https://formulae.brew.sh/formula/git))
1. Clone Repository ´git clone https://github.com/atopile/swoop.git´
1. Install [Visual Studio Code](https://code.visualstudio.com/) Hombrew command: `brew install --cask visual-studio-code` ([formula](https://formulae.brew.sh/cask/visual-studio-code))
1. Install [PlatformIO Visual Studio Code Extension](https://marketplace.visualstudio.com/items?itemName=platformio.platformio-ide)
1. Reopen Visual Studio Code and open the folder of the repository, it will install all dependencies
1. At the bottom left of the window you find buttons for building, uploading, serial telemetry
