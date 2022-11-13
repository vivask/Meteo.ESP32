# ESP32 meteo station

Implemented support for BME280, DS18B20, Radsense, ZE18CH2O sensors. It is possible to use the MICS6814 sensor connected via arduino, part of the acduin code is located here https://github.com/vivask/Meteo.Avr. The project involves the use of backend https://github.com/vivask/Meteo.Backend.

Requires pre-installation and configuration of esp idf https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/vscode-setup.html

## Hardware Required

To run this example, you need an ESP32 dev board (e.g. ESP32-WROVER Kit, ESP32-Ethernet-Kit) or ESP32 core board (e.g. ESP32-DevKitC). 

### Pin Assignment:

The following pin connection is used in this example.

| ESP32  | BME280  | RADSENS | DS18B20 | ZE18CH2O | ARDUINO |
| ------ | ------- | ------- | ------- | -------- | ------- |
| GPIO22 | SCL     | SCL     |         |          |         |
| GPIO22 | SDA     | SDA     |         |          |         |
| GPIO21 |         |         | OUT     |          |         |
| GPIO19 |         |         |         | TX       |         |
| GPIO18 |         |         |         |          | TX      |


### Configure the project
```sh
~cd /tmp
~git clone https://github.com/vivask/Meteo.ESP32.git
~cd Meteo.ESP32
~idf.py menuconfig
```
More information about project configuration here https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/kconfig.html

### Build and Flash

Run `idf.py -p PORT flash monitor` to build and flash the project..

(To exit the serial monitor, type ``Ctrl-]``.)

See the [Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html) for full steps to configure and use ESP-IDF to build projects.

### Configure meteo station
After successful firmware and restart of the controller, a Wi-Fi access point with the name es32 will be available. You need to connect to it, then use any browser to go to the address http://10.10.0.1

The specified parameters are valid for the controller's default configuration.