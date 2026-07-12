{
  description = "FAST-LIVO2 native module (LCM glue over unmodified upstream)";

  inputs = {
    # Pinned to the same rev as the pointlio module's lock so the PCL/OpenCV
    # dep chain hits the binary cache instead of rebuilding from source.
    nixpkgs.url = "github:NixOS/nixpkgs/ec7c70d12ce2fc37cb92aff673dcdca89d187bae";
    flake-utils.url = "github:numtide/flake-utils";
    dimos-lcm = {
      url = "github:dimensionalOS/dimos-lcm/main";
      flake = false;
    };
    lcm-extended = {
      url = "github:jeff-hykin/lcm_extended";
      inputs.nixpkgs.follows = "nixpkgs";
      inputs.flake-utils.follows = "flake-utils";
    };
    fast-livo2 = {
      # Upstream, compiled unmodified — all ROS decoupling happens via the
      # fake-ROS shim headers in ./shim (see CMakeLists.txt).
      url = "github:hku-mars/FAST-LIVO2/0d2c0346107b75b59934975adec9a6eeeb913c64";
      flake = false;
    };
    sophus-old = {
      # The non-templated Sophus revision FAST-LIVO2 pins in its README §2.3.
      url = "github:strasdat/Sophus/a621ff2e56c56c839a6c40418d42c3c254424b5c";
      flake = false;
    };
    vikit = {
      # FAST-LIVO2's camera-model fork of rpg_vikit (vikit_common only is used).
      url = "github:xuankuzcr/rpg_vikit/6c886c8e5d83997806e00294826d528cea3581dd";
      flake = false;
    };
  };

  outputs = { self, nixpkgs, flake-utils, dimos-lcm, lcm-extended, fast-livo2, sophus-old, vikit, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };
        lcm = lcm-extended.packages.${system}.lcm;

        fastlivo_native = pkgs.stdenv.mkDerivation {
          pname = "fastlivo_native";
          version = "0.1.0";

          src = ./.;

          nativeBuildInputs = [ pkgs.cmake pkgs.pkg-config ];
          buildInputs = [
            lcm
            pkgs.glib
            pkgs.eigen
            pkgs.pcl
            pkgs.opencv
            pkgs.boost
            pkgs.llvmPackages.openmp
          ];

          cmakeFlags = [
            "-DCMAKE_POLICY_VERSION_MINIMUM=3.5"
            "-DFETCHCONTENT_SOURCE_DIR_DIMOS_LCM=${dimos-lcm}"
            "-DFASTLIVO2_DIR=${fast-livo2}"
            "-DSOPHUS_DIR=${sophus-old}"
            "-DVIKIT_DIR=${vikit}"
          ];
        };
      in {
        packages = {
          default = fastlivo_native;
          inherit fastlivo_native;
        };
      });
}
