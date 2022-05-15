#!/bin/bash

set -e

# TODO: For some reason I need to install this: https://packages.ubuntu.com/impish/amd64/libssl1.1/download

cargo build --release
espflash /dev/ttyUSB0 target/xtensa-esp32-espidf/release/flightctrl
RUST_BACKTRACE=1 espmonitor  --chip esp32 /dev/ttyUSB0
