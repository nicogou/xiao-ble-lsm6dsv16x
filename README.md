# LSM6DSV16X evaluation

<a href="https://nrfconnect.github.io/ncs-example-application">
  <img alt="Documentation" src="https://img.shields.io/badge/documentation-3D578C?logo=sphinx&logoColor=white">
</a>
<a href="https://nrfconnect.github.io/ncs-example-application/doxygen">
  <img alt="API Documentation" src="https://img.shields.io/badge/API-documentation-3D578C?logo=c&logoColor=white">
</a>

This repository contains an application aimed at evaluating the LSM6DSV16X IMU in the context of surfing.
The end goal of the device is to be able to detect, analyze and provide numerical data related to surfing sessions.

## Hardware overview

The device is based around:
- the Xiao BLE Sense from Seeed Studio, a nRF52840-based development board containing a 2MB QSPI Flash, a microphone and a basic IMU.
- the LSM6DSV16X IMU from STMicroelectronics, a smart IMU with an embedded Qvar (electrostatic charge) sensor, able to perform sensor fusion and edge computing.

Early firmware development and product evaluation is done using breakout boards, but it is planned to develop a custom PCB if these tests are successful.

## Getting started

Before getting started, make sure you have a proper nRF Connect SDK development environment.
Follow the official
[Installation guide](https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrf/installation/install_ncs.html).

### Initialization

The first step is to initialize the workspace folder (``my-workspace``) where
the ``example-application`` and all nRF Connect SDK modules will be cloned. Run the following
command:

```shell
# initialize my-workspace for the application (main branch)
west init -m https://github.com/nicogou/xiao-ble-lsm6dsv16x.git --mr main my-workspace
# update nRF Connect SDK modules
cd my-workspace
west update
```

### Building and running

To build the application, run the following command:

```shell
cd xiao-ble-lsm6dsv16x.git
west build app
```

A sample debug configuration is also provided. To apply it, run the following
command:

```shell
west build app -- -DEXTRA_CONF_FILE="debug.conf"
```

In order to flash the board, it needs to be in bootloader mode. To do this, quickly double press the RESET button present on the Xiao BLE Sense.
Once you have built the application, run the following command to flash it:

```shell
west flash -r uf2
```

## Testing

To execute Twister integration tests, run the following command:

```shell
west twister -T tests --integration
```

## Documentation

A minimal documentation setup is provided for Doxygen and Sphinx. To build the
documentation first change to the ``doc`` folder:

```shell
cd doc
```

Before continuing, check if you have Doxygen installed. It is recommended to
use the same Doxygen version used in [CI](.github/workflows/docs.yml). To
install Sphinx, make sure you have a Python installation in place and run:

```shell
pip install -r requirements.txt
```

API documentation (Doxygen) can be built using the following command:

```shell
doxygen
```

The output will be stored in the ``_build_doxygen`` folder. Similarly, the
Sphinx documentation (HTML) can be built using the following command:

```shell
make html
```

The output will be stored in the ``_build_sphinx`` folder. You may check for
other output formats other than HTML by running ``make help``.
