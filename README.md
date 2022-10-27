# esp8266-at-driver

[![CI](https://github.com/drogue-iot/esp8266-at-driver/actions/workflows/ci.yaml/badge.svg)](https://github.com/drogue-iot/esp8266-at-driver/actions/workflows/ci.yaml)
[![crates.io](https://img.shields.io/crates/v/esp8266-at-driver.svg)](https://crates.io/crates/esp8266-at-driver)
[![docs.rs](https://docs.rs/esp8266-at-driver/badge.svg)](https://docs.rs/esp8266-at-driver)
[![Matrix](https://img.shields.io/matrix/drogue-iot:matrix.org)](https://matrix.to/#/#drogue-iot:matrix.org)

Driver for the esp8266-wifi adapter with AT command firmware. The adapter has a serial interface, therefore this driver can be used with any UART driver that implements the `embedded-io` traits.

## Features

* Implements `embedded-nal-async` traits
* Implements `embedded-io` traits
* Full async support, based on `embassy` libraries

## Examples

See [examples/linux](examples/linux) for an example that works in a Linux environment.

