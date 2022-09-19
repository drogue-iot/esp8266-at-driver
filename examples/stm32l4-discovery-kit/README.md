# STM32 IOT01A example

This example application runs out of the box on the STM32 IOT01A development kits. 

It uses the on board WiFi and the temperature sensor to send data to a TCP server and expects to receive the same data back (The typical echo server).

NOTE: There are multiple variants of this kit, so the example might need modifications.

=== Prerequisites

==== Hardware

* STM32 IOT01A development kit

==== Software

* To build the example, you need to have link:https://rustup.rs/[rustup].
* To flash the example on the device, you need `probe-run` installed (`cargo install probe-run`).

=== Configuring

Modify `main.rs` to use credentials for your local WiFi network.

=== Running

To run the firmware using a given chip:

----
cargo run --release
----

== Troubleshooting

If you’re experiencing problems, try setting the `VID:PID` values to that of your probe (you can find that from lsusb once your board is powered).

....
<ENV> cargo run <ARGS> -- --probe <VID>:<PID>
....
