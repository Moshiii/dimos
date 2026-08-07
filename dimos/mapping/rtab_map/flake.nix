{
  description = "dimos RTAB-Map native C++ module";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    lcm-extended = {
      url = "github:jeff-hykin/lcm_extended";
      inputs.nixpkgs.follows = "nixpkgs";
      inputs.flake-utils.follows = "flake-utils";
    };
    # Generated LCM message headers, consumed via a FetchContent source override.
    dimos-lcm = {
      url = "github:dimensionalOS/dimos-lcm/main";
      flake = false;
    };
    # Standalone Boost.PFR, consumed by the dimos native SDK the same way.
    pfr = {
      url = "github:apolukhin/pfr_non_boost/2.3.2";
      flake = false;
    };
  };

  outputs = { self, nixpkgs, flake-utils, lcm-extended, dimos-lcm, pfr, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };
        lcm = lcm-extended.packages.${system}.lcm;
      in {
        packages.default = pkgs.stdenv.mkDerivation {
          pname = "dimos-rtab-map";
          version = "0.1.0";
          src = ./.;

          nativeBuildInputs = [ pkgs.cmake pkgs.pkg-config ];

          # nixpkgs' rtabmap is built with the Qt GUI, so qtbase rides in on its
          # buildInputs and the Qt setup hook then demands a wrapping decision.
          # This binary is a headless module with no GUI to wrap.
          dontWrapQtApps = true;
          # rtabmap's exported CMake config re-runs find_package for every optional
          # dependency it was built against (PCL, OpenCV, g2o, octomap, Qt, ...), so
          # consuming it means having that whole set present. Inheriting rtabmap's
          # own buildInputs is what guarantees the set matches the one it was built
          # with, rather than a list here drifting out of sync with nixpkgs.
          buildInputs = [
            pkgs.rtabmap
            lcm
            pkgs.glib
            pkgs.nlohmann_json
          ] ++ pkgs.rtabmap.buildInputs;

          cmakeFlags = [
            "-DCMAKE_BUILD_TYPE=Release"
            "-DCMAKE_POLICY_VERSION_MINIMUM=3.5"
            "-DFETCHCONTENT_SOURCE_DIR_DIMOS_LCM=${dimos-lcm}"
            "-DFETCHCONTENT_SOURCE_DIR_PFR=${pfr}"
            # Header-only dimos native SDK lives outside this dir; a git-tree
            # flake can reach it as a path literal within the repo tree.
            "-DDIMOS_NATIVE_CPP_DIR=${../../../native/cpp}"
          ];
        };

        devShells.default = pkgs.mkShell {
          inputsFrom = [ self.packages.${system}.default ];
        };
      });
}
