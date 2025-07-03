{ pkgs ? import <nixpkgs> {}}:
   pkgs.pkgsCross.riscv32-embedded.mkShell {
     buildInputs = [];
     nativeBuildInputs = [
      pkgs.pkgsCross.riscv32-embedded.buildPackages.gdb
    ];
   }
