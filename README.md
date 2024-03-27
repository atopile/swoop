# swoop

Swoop is a motion controller for FPV drone. We'll add more details here as we go.

## Architecture

The project contains two subsystems

1) RP2040, sensors and IOs. Those are meant to capture the inputs of the user and generate the control outputs.
2) ELRS module. This module is meant to communicate with the aircraft's receiver.

![IMG_7410](https://github.com/atopile/swoop/assets/9785003/696a1b3a-4fe3-41b3-800c-c4fb4323561a)

