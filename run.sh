#!/usr/bin/env bash

set -eux

USB=${1:-/dev/ttyUSB0}
export DEFMT_LOG=${DEFMT_LOG:-trace}
LOG_FORMAT=${LOG_FORMAT:-'[{L}]{f:>10}:{l:<4}: {s}'}

EXE=vapor_keeb
OUT_DIR=target/riscv32imafc-unknown-none-elf/release

cargo flash --release --chip CH32V307
riscv32-elf-objdump -dC $OUT_DIR/$EXE > $OUT_DIR/$EXE.objdump || echo 'riscv32-elf-objdump not found, skipping OBJDUMP'
socat $USB,rawer,b115200 STDOUT | defmt-print --verbose -e $OUT_DIR/$EXE --log-format "$LOG_FORMAT"
