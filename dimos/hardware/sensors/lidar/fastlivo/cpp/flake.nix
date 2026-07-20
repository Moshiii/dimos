{
  description = "FastLivo native binary — built from jeff-hykin/dimos-module-fastlivo";

  inputs = {
    flake-utils.url = "github:numtide/flake-utils";
    # All sources (LCM glue, fake-ROS shims, pinned upstream FAST-LIVO2 chain)
    # live in the module repo; this flake just re-exports its package.
    dimos-module-fastlivo.url = "github:jeff-hykin/dimos-module-fastlivo/main";
  };

  outputs = { self, flake-utils, dimos-module-fastlivo }:
    flake-utils.lib.eachDefaultSystem (system: {
      packages = {
        inherit (dimos-module-fastlivo.packages.${system}) fastlivo_native;
        default = dimos-module-fastlivo.packages.${system}.fastlivo_native;
      };
    });
}
