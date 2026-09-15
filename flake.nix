{
  description = "libvc1 - experimental VC-1 codec library and vc1enc frontend";

  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";

  outputs = { self, nixpkgs }:
    let
      systems = [ "x86_64-linux" "aarch64-linux" ];
      libvc1Version = builtins.replaceStrings [ "\n" "\r" ] [ "" "" ] (builtins.readFile ./VERSION);
      forAllSystems = f: nixpkgs.lib.genAttrs systems (system: f system (import nixpkgs { inherit system; }));
    in {
      packages = forAllSystems (system: pkgs:
        let
          mkLibvc1 = { debug ? false, simdTestMode ? "native" }: pkgs.stdenv.mkDerivation {
            pname = if debug then "libvc1-debug"
              else if simdTestMode == "qemu-fallback" then "libvc1-qemu-tests"
              else if simdTestMode == "qemu-force" then "libvc1-qemu-force-tests"
              else "libvc1";
            version = libvc1Version;
            src = self;

            strictDeps = true;
            nativeBuildInputs = [
              pkgs.bash
              pkgs.cmake
              pkgs.ninja
              pkgs.ffmpeg
              pkgs.glslang
              (pkgs.python3.withPackages (ps: [ ps.numpy ]))
            ] ++ pkgs.lib.optionals (simdTestMode != "native") [ pkgs.qemu ];
            buildInputs = [
              pkgs.libebml
              pkgs.libmatroska
              pkgs.vulkan-headers
              pkgs.vulkan-loader
            ];

            configurePhase = ''
              runHook preConfigure
              cmake -S . -B build -G Ninja \
                -DCMAKE_BUILD_TYPE=${if debug then "Debug" else "Release"} \
                -DLIBVC1_ENABLE_SANITIZERS=${if debug then "ON" else "OFF"} \
                -DLIBVC1_SIMD_TEST_MODE=${simdTestMode} \
                -DCMAKE_INSTALL_PREFIX="$out"
              runHook postConfigure
            '';

            buildPhase = ''
              runHook preBuild
              cmake --build build --parallel "$NIX_BUILD_CORES"
              runHook postBuild
            '';

            doCheck = true;
            checkPhase = if debug then ''
              runHook preCheck
              export ASAN_OPTIONS="detect_leaks=1:halt_on_error=1"
              export UBSAN_OPTIONS="halt_on_error=1:print_stacktrace=1"
              # Keep the sanitizer package practical to build: exercise the
              # SIMD dispatch/safety tests plus the public API. The normal
              # package continues to run the complete CTest + smoke suite.
              ctest --test-dir build --output-on-failure \
                -R 'libvc1-(simd-chroma-edge-guard-v3|simd-chroma-edge-guard-v4|simd-forward-transform-v3|simd-forward-transform-v4|simd-dispatch-runtime|api-c$|api-cpp$)'
              runHook postCheck
            '' else ''
              runHook preCheck
              ctest --test-dir build --output-on-failure
              bash ./tests/smoke.sh ./build/vc1enc
              runHook postCheck
            '';

            installPhase = ''
              runHook preInstall
              cmake --install build
              env -u LD_LIBRARY_PATH "$out/bin/vc1enc" --help >/dev/null
              runHook postInstall
            '';

            meta = {
              description = if debug
                then "Experimental VC-1 codec library debug build with ASan and UBSan"
                else "Experimental VC-1 codec library with vc1enc M2TS/ASF/Matroska frontend";
              license = pkgs.lib.licenses.lgpl21Plus;
              platforms = pkgs.lib.platforms.linux;
              mainProgram = "vc1enc";
            };
          };
        in ({
          default = mkLibvc1 { };
          debug = mkLibvc1 { debug = true; };
        } // pkgs.lib.optionalAttrs (system == "x86_64-linux") {
          qemu-tests = mkLibvc1 { simdTestMode = "qemu-fallback"; };
          qemu-force-tests = mkLibvc1 { simdTestMode = "qemu-force"; };
        }));

      apps = forAllSystems (system: pkgs: {
        default = {
          type = "app";
          program = "${self.packages.${system}.default}/bin/vc1enc";
        };
      });

      devShells = forAllSystems (system: pkgs: {
        default = pkgs.mkShell {
          packages = [
            pkgs.bash
            pkgs.cmake
            pkgs.ninja
            pkgs.gcc
            pkgs.libebml
            pkgs.libmatroska
            pkgs.ffmpeg
            pkgs.vulkan-headers
            pkgs.vulkan-loader
            pkgs.glslang
            pkgs.qemu
            (pkgs.python3.withPackages (ps: [ ps.numpy ]))
          ];
        };
      });
    };
}
