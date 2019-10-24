# Bluetooth low energy sensor experiments

These are my attempts to make a Bluetooth low energy based sensor system using the Nordic nRF52840 dongle (PCA10059). At some point support for receiving readings from the Xiaomi Mijia temperature and humidity sensor was also added.

This project is functional but still a work in progress.

Bluetooth and USB communication uses Protobuf for serialization and a PC client with a Prometheus exporter is provided.

## Development environment

Development was done using Black Magic Probe firmware flashed onto STM bluepills so the Makefiles will reflect that.

Toolchain: `gcc-arm-none-eabi-8-2018-q4-major`

SDK: `nRF5_SDK_15.3.0_59ac345`

You will have to change at least `GNU_INSTALL_ROOT`, `SDK_ROOT` and `BLACKMAGIC_TARGET` in the makefiles to get the code to build.

### Black Magic Probes and the PCA10059

Current Black Magic Probes (Firmware v1.6.1-363-g90df817) will reset the nRF52840 when performing the `erase_mass` action. This will cause the nRF52840 to come up with 1.8V io and will not be flashable with a bluepill running Black Magic Probe. To get around this I use a TXS0108E breakout board for level conversion with two of the pins driving the swdclk line. The two pins are needed to reliably overcome the ~10K pull down on the swdclk line.

## Components

This project currently contains the following components:

### ble_sensor_coded

Firmware for the PCA10059 to read temperatures from 1-wire probes and make it available using a notify characteristic. This is intended to be run on a battery powered dongle.

Build and flash with

``` bash
make MORECFLAGS="-DNRF_LOG_ENABLED=1 -DDEBUG" blackmagicflash -j12
```

for a debug build and

``` bash
make blackmagicflash -j12
```

Debug output is via UART on pin 0.31. Enabling debugging significantly increases the power consumption.

### ble_app_sensor_central

Firmware for the PCA10059 to receive sensor readings from the characteristic exposed by `ble_sensor_coded`. As a bonus it will also receive temperature and humidity readings from a Xiaomi Mijia temperature and humidity sensor.

Readings are exposed over USB using the CDC ACM class.

Build with

``` bash
make blackmagicflash -j12
```

Debug output is via UART on pin 0.31.

### ble_experiment_exporter

Reads values over CDC ACM and exposes them as Prometheus metrics. Only tested on Linux.

Build with `make`.

## Licenses

Code based on Nordic examples retain their 5 clause license. Original code written for this is licensed under 3 clause BSD. See dependency folders for their license details.
