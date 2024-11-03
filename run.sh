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
EXTRA=
DEBUG="--release"

while getopts "du:b:h" opt; do
  case $opt in
    d)
      DEBUG=""
      ;;
    u)
      UART="$OPTARG"
      ;;
    b)
      EXE="$OPTARG"
      ;;
    h)
      EXTRA+=" --features usbhs"
      ;;
    \?)
      echo "Invalid option: -$OPTARG" >&2
      exit 1
      ;;
  esac
done

if [[ -z $DEBUG ]] ; then
OUT_DIR=target/riscv32imac-unknown-none-elf/debug
else
OUT_DIR=target/riscv32imac-unknown-none-elf/release
fi

if [ "${CAT:-}" == "" ] ; then
cargo build $DEBUG --bin "$EXE" $EXTRA
$OBJDUMP -dC $OUT_DIR/$EXE > $OUT_DIR/$EXE.objdump || echo "$OBJDUMP not found, skipping OBJDUMP"
# probe-rs download --chip CH32V307 $OUT_DIR/$EXE
# probe-rs reset --chip CH32V307
wlink flash $OUT_DIR/$EXE
fi

serial-cat $UART | defmt-print --verbose -e $OUT_DIR/$EXE --log-format "$LOG_FORMAT" | tee $LOG_FILE
