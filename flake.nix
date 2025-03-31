{
  inputs = {
    nixpkgs.url = "nixpkgs/nixos-unstable";

    utils.url = "github:numtide/flake-utils";

    commonroad-cmake = {
      url = "git+https://gitlab.lrz.de/tum-cps/commonroad-cmake.git?ref=main";
      flake = false;
    };
  };

  outputs =
    { self
    , nixpkgs
    , ...
    } @ inputs:
    let
      supportedSystems = [
        "x86_64-linux"
        "i686-linux"
        "aarch64-linux"
        "x86_64-darwin"
      ];
    in
    inputs.utils.lib.eachSystem supportedSystems (system:
    let
      pkgs = import nixpkgs {
        inherit system;
      };
    in
    rec {
      formatter = pkgs.nixpkgs-fmt;

      packages = rec {
        inherit
          (pkgs.callPackages ./commonroad_deps.nix { })
          commonroad_vehicle_models
          commonroad_io
          ;

        environment-model = pkgs.callPackage ./default.nix {
          inherit (inputs) commonroad-cmake;
          boost = pkgs.boost187;
          # enableSanitizers = true;

          inherit commonroad_io;
        };
        environment-model-python =
          let
            pythonPkgs = pkgs.python3Packages;
          in
          environment-model.override {
            enablePython = true;
            enableDocs = false;
            pythonPackages = pythonPkgs;
            enableSanitizers = false;
          };

        default = environment-model;
      };

      devShells = {
        default = pkgs.mkShell {
          inputsFrom = [ packages.environment-model ];

          packages = with pkgs; [
            doxygen
            python312Packages.pip
            cmake
            ninja

            clang-tools

            graphviz
          ];
        };

        mkdoxy = pkgs.mkShell {
          buildInputs = with pkgs; [ mkdoxy ];
        };

        clangd = pkgs.mkShell {
          inputsFrom = [ packages.environment-model.override { stdenv = pkgs.clang14Stdenv; } ];
          packages = with pkgs; [ clang-tools_14 llvmPackages_14.openmp ];
          shellHook = ''
            export CC=${pkgs.clang14Stdenv.cc}/bin/clang
            export CXX=${pkgs.clang14Stdenv.cc}/bin/clang++

            mkdir -p .build-clangd-compile-commands
            cd .build-clangd-compile-commands
            cmake --fresh -DCMAKE_EXPORT_COMPILE_COMMANDS=ON ${pkgs.lib.concatStringsSep " " packages.environment-model.cmakeFetchContentFlags} ..
            cd ..
            ln -sf .build-clangd-compile-commands/compile_commands.json .

          '';
        };
      };
    });
}
