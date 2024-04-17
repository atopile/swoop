## Overview

The ESP32-S3 is a powerful, versatile microcontroller designed for IoT applications. It features dual-core Xtensa LX7 processors, Wi-Fi and Bluetooth 5 connectivity, rich I/O capabilities, and enhanced security features, making it ideal for smart home devices, wearable electronics, and automated systems requiring efficient processing and wireless communication.

![ESP32-S3](https://firebasestorage.googleapis.com/v0/b/atopile.appspot.com/o/esp32-s3.png?alt=media&token=7047e466-db4b-4756-bb38-15589d830f78 "ESP32-S3")

## Usage Example

```
import ESP32 from "esp32/esp32.ato"
import Power from "generics/interfaces.ato"

module my_project:
   power3v3 = new Power
   micro = new ESP32
   micro.power ~ power3v3

```

## Features
Power interface: 3.3V input
SPI interface: 4-wire SPI
I2C interface
JTAG interface

## Contributing
Contribute to this package using pull requests.

## License
This battery connector module is provided under the MIT License.

## Contact
For further inquiries or support, please contact me at narayan@atopile.io.


