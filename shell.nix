{ pkgs ? import <nixpkgs> {}}:
   pkgs.mkShell {
     nativeBuildInputs = [
      pkgs.lld
      pkgs.pkgsCross.riscv32-embedded.buildPackages.gdb
      pkgs.pkgsCross.riscv32-embedded.buildPackages.gcc
      ];
      shellHooks = ''
        alias riscv32-unknown-elf-gcc='riscv32-none-elf-gcc'
        alias riscv32-unknown-elf-g++='riscv32-none-elf-g++'
      '';
   }
