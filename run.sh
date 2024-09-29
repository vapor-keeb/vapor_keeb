#!/usr/bin/env bash

set -eux

USB=${1:-/dev/ttyUSB0}

cargo flash --release --chip CH32V307
socat $USB,rawer,b115200 STDOUT | defmt-print -e target/riscv32imafc-unknown-none-elf/release/vapor_keeb
