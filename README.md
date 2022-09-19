# es-wifi-driver

[![CI](https://github.com/drogue-iot/es-wifi-driver/actions/workflows/ci.yaml/badge.svg)](https://github.com/drogue-iot/es-wifi-driver/actions/workflows/ci.yaml)
[![crates.io](https://img.shields.io/crates/v/es-wifi-driver.svg)](https://crates.io/crates/es-wifi-driver)
[![docs.rs](https://docs.rs/es-wifi-driver/badge.svg)](https://docs.rs/es-wifi-driver)
[![Matrix](https://img.shields.io/matrix/drogue-iot:matrix.org)](https://matrix.to/#/#drogue-iot:matrix.org)

Driver for the es-wifi adapter from Inventek. The adapter is used on boards such as [B-L475E-IOT01A](https://www.st.com/en/evaluation-tools/b-l475e-iot01a.html) and [B-L4S5I-IOT01A](https://www.st.com/en/evaluation-tools/b-l4s5i-iot01a.html).

## Features

* Implements `embedded-nal-async` traits
* Implements `embedded-io` traits
* Full async support, based on `embassy` libraries

## Examples

See [examples/stm32l4-discovery-kit](examples/stm32l4-discovery-kit) for an example that works the B-L475E-IOT01A board.

