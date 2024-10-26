#!/usr/bin/env bash

set -eux

OS=$(uname)
UART="/dev/ttyUSB0"
export DEFMT_LOG=${DEFMT_LOG:-trace}
LOG_FORMAT=${LOG_FORMAT:-'[{L}]{f:>10}:{l:<4}: {s}'}
LOG_FILE="usb.log"

OBJDUMP=riscv32-elf-objdump

if [[ ! -x $(which $OBJDUMP) ]]; then
echo 'using rv64 objdump'
OBJDUMP=riscv64-elf-objdump
fi

EXE=vapor_keeb
OUT_DIR=target/riscv32imac-unknown-none-elf/release

while getopts "u:b:" opt; do
  case $opt in
    u)
      UART="$OPTARG"
      ;;
    b)
      BIN_NAME="$OPTARG"
      EXE="$BIN_NAME"
      ;;
    \?)
      echo "Invalid option: -$OPTARG" >&2
      exit 1
      ;;
  esac
done

if [ "${CAT:-}" == "" ] ; then
if [[ -n "${BIN_NAME+x}" ]]; then
cargo build --release --bin "$BIN_NAME"
else
cargo build --release
fi
$OBJDUMP -dC $OUT_DIR/$EXE > $OUT_DIR/$EXE.objdump || echo "$OBJDUMP not found, skipping OBJDUMP"
# probe-rs download --chip CH32V307 $OUT_DIR/$EXE
# probe-rs reset --chip CH32V307
wlink flash $OUT_DIR/$EXE
fi

serial-cat $UART | defmt-print --verbose -e $OUT_DIR/$EXE --log-format "$LOG_FORMAT" | tee $LOG_FILE
