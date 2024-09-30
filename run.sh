#!/usr/bin/env bash

set -eux

USB=${1:-/dev/ttyUSB0}
export DEFMT_LOG=${DEFMT_LOG:-trace}
LOG_FORMAT=${LOG_FORMAT:-'[{L}]{f:>10}:{l:<4}: {s}'}

cargo flash --release --chip CH32V307
socat $USB,rawer,b115200 STDOUT | defmt-print --verbose -e target/riscv32imafc-unknown-none-elf/release/vapor_keeb --log-format "$LOG_FORMAT"
