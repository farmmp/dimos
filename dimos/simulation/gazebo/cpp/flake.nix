{
  description = "Gazebo Ionic (gz-sim 9) – standalone, no ROS, true headless via EGL";

  inputs = {
    # nixos-24.11: ships protobuf 25, modern OGRE 14, and the libGL stack
    # gz-rendering 9 needs.
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-24.11";
    # nixos-23.05 still ships ogre1_10. gz-rendering 9's `ogre` engine
    # subdir still uses the legacy OGRE 1.x API (Camera::yaw/pitch/roll,
    # _suppressRenderStateChanges, etc.) so we still need 1.10 here.
    # gz-rendering 9's `ogre2` engine wants OGRE-Next 2.3.x which isn't
    # packaged in nixpkgs at all — packaging that from source is a
    # several-hour task on its own.
    nixpkgs-old.url = "github:NixOS/nixpkgs/nixos-23.05";
    flake-utils.url = "github:numtide/flake-utils";
    dimos-lcm = {
      url = "github:dimensionalOS/dimos-lcm/main";
      flake = false;
    };
  };

  outputs = { self, nixpkgs, nixpkgs-old, flake-utils, dimos-lcm, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          config.permittedInsecurePackages = [
            "freeimage-unstable-2021-11-01"
            "freeimage-3.18.0-unstable-2024-04-18"
          ];
        };

        pkgs-old = import nixpkgs-old {
          inherit system;
          config.permittedInsecurePackages = [
            "freeimage-unstable-2021-11-01"
            "freeimage-3.18.0-unstable-2024-04-18"
          ];
        };
        ogre = pkgs-old.ogre1_10;

        # ---------- helper: every gz lib is a cmake project --------
        mkGzPkg = { pname, version, src, buildInputs ? [], cmakeFlags ? [],
                     nativeBuildInputs ? [], preConfigure ? "", postInstall ? "",
                     preFixup ? "", patches ? [], ... }:
          pkgs.stdenv.mkDerivation {
            inherit pname version src patches preConfigure postInstall;
            nativeBuildInputs = [ pkgs.cmake pkgs.pkg-config ] ++ nativeBuildInputs;
            buildInputs = buildInputs;
            cmakeFlags = [
              "-DCMAKE_POLICY_VERSION_MINIMUM=3.5"
              "-DBUILD_TESTING=OFF"
            ] ++ cmakeFlags;
            # Strip /build/ refs that some plugins leave in their RPATH
            preFixup = ''
              find $out -type f \( -name '*.so' -o -name '*.so.*' \) -print0 \
                | while IFS= read -r -d "" f; do
                    rp=$(patchelf --print-rpath "$f" 2>/dev/null || true)
                    if [ -n "$rp" ] && echo "$rp" | grep -q '/build/'; then
                      new_rp=$(echo "$rp" | tr ':' '\n' | grep -v '/build/' | paste -sd: -)
                      patchelf --set-rpath "$new_rp" "$f"
                    fi
                  done
            '' + preFixup;
          };

        # Transitive deps that gz cmake configs require at configure time
        transitiveDeps = [
          pkgs.protobuf pkgs.python3 pkgs.tinyxml-2 pkgs.zeromq
          pkgs.cppzmq pkgs.libuuid pkgs.zlib pkgs.eigen pkgs.gdal
          pkgs.freeimage pkgs.curl pkgs.jsoncpp pkgs.libzip
          pkgs.spdlog pkgs.cli11  # gz-utils3 transitive (log + cli components)
        ];

        # ==================== sources (Ionic) ======================

        gz-cmake-src = pkgs.fetchFromGitHub {
          owner = "gazebosim"; repo = "gz-cmake";
          rev = "gz-cmake4_4.2.1";
          hash = "sha256-zhpZnvfnWsuyykIbNB4xgHxdO35otmaz0x/VXSdWPNY=";
        };

        gz-utils-src = pkgs.fetchFromGitHub {
          owner = "gazebosim"; repo = "gz-utils";
          rev = "gz-utils3_3.1.1";
          hash = "sha256-fYzysdB608jfMb/EbqiGD4hXmPxcaVTUrt9Wx0dBlto=";
        };

        gz-math-src = pkgs.fetchFromGitHub {
          owner = "gazebosim"; repo = "gz-math";
          rev = "gz-math8_8.3.0";
          hash = "sha256-qXce3btwZn/iZoLFCWMWJGv/AK0RgIYx6zbKoXdHjzY=";
        };

        sdformat-src = pkgs.fetchFromGitHub {
          owner = "gazebosim"; repo = "sdformat";
          rev = "sdformat15_15.4.0";
          hash = "sha256-zMCoWFPUN/Q7J5F6mILoL/Ttgf/KXqenj6umlXqxZ90=";
        };

        gz-common-src = pkgs.fetchFromGitHub {
          owner = "gazebosim"; repo = "gz-common";
          rev = "gz-common6_6.3.0";
          hash = "sha256-9IsV8Mc6evJAO+5tXNdb0j3kmieR96e/OKLNXii3WKk=";
        };

        gz-plugin-src = pkgs.fetchFromGitHub {
          owner = "gazebosim"; repo = "gz-plugin";
          rev = "gz-plugin3_3.1.0";
          hash = "sha256-3La9TqxljV1Lko6ju+b8CCspDbhXGPLOGMivqYElTXM=";
        };

        gz-msgs-src = pkgs.fetchFromGitHub {
          owner = "gazebosim"; repo = "gz-msgs";
          rev = "gz-msgs11_11.1.0";
          hash = "sha256-M/rzUrL6uzpaRNLWJsGViY6Jk0bLtooEe+0eEEPS7PA=";
        };

        gz-transport-src = pkgs.fetchFromGitHub {
          owner = "gazebosim"; repo = "gz-transport";
          rev = "gz-transport14_14.2.0";
          hash = "sha256-jvEVa0BK7hnYWybNXh30KpNu00+OTtR9bdHCiN8Bpeg=";
        };

        gz-fuel-tools-src = pkgs.fetchFromGitHub {
          owner = "gazebosim"; repo = "gz-fuel-tools";
          rev = "gz-fuel-tools10_10.1.0";
          hash = "sha256-ONo0zmKHSu1i6GAouDzFD5T2PUNXJ4IjhgPSoORRzao=";
        };

        gz-rendering-src = pkgs.fetchFromGitHub {
          owner = "gazebosim"; repo = "gz-rendering";
          rev = "gz-rendering9_9.5.0";
          hash = "sha256-oinqpmtQt7DlpLvkb4xlXh2vprJqGaxh9LC1NLDiyXQ=";
        };

        gz-gui-src = pkgs.fetchFromGitHub {
          owner = "gazebosim"; repo = "gz-gui";
          rev = "gz-gui9_9.0.2";
          hash = "sha256-2HA9Ah2QdC9VmYAJdC/36Kiin9lJJbSOT9YlJj3VwU8=";
        };

        gz-physics-src = pkgs.fetchFromGitHub {
          owner = "gazebosim"; repo = "gz-physics";
          rev = "gz-physics8_8.3.0";
          hash = "sha256-U02OIZ59IMxxbZeC8bjqmFKmfWTzDTc7F4YO5gsJdYg=";
        };

        gz-sensors-src = pkgs.fetchFromGitHub {
          owner = "gazebosim"; repo = "gz-sensors";
          rev = "gz-sensors9_9.2.0";
          hash = "sha256-Vxl3xdmh8ybRbjDxNGt8qgQOP9ctAcYAoVwWeytAglc=";
        };

        gz-sim-src = pkgs.fetchFromGitHub {
          owner = "gazebosim"; repo = "gz-sim";
          rev = "gz-sim9_9.5.0";
          hash = "sha256-qUnItGpZkE4HTufhO/gBefX5AfHd2jfnWEwGwYmlKIE=";
        };

        # ==================== packages (build order) ===============

        gz-cmake = mkGzPkg {
          pname = "gz-cmake";
          version = "4.2.1";
          src = gz-cmake-src;
          # Patch FindGzOGRE.cmake to accept OGRE 13+ (the modern nixpkgs
          # `ogre` is v14, the renumbered continuation of the 1.x API line —
          # same headers, same OgreMain target — but the upstream module
          # explicitly rejects anything >= 2.0). Treat 2.0..12.x as the
          # OGRE-Next / 2.x line that we don't want; allow 13+ through.
          postInstall = ''
            f=$out/share/cmake/gz-cmake4/cmake4/FindGzOGRE.cmake
            if [ -f "$f" ]; then
              substituteInPlace "$f" --replace-fail \
                'if (NOT ''${OGRE_VERSION} VERSION_LESS 2.0.0)' \
                'if (NOT ''${OGRE_VERSION} VERSION_LESS 2.0.0 AND ''${OGRE_VERSION} VERSION_LESS 13.0.0)'
            fi
          '';
        };

        gz-utils = mkGzPkg {
          pname = "gz-utils";
          version = "3.1.1";
          src = gz-utils-src;
          buildInputs = [ gz-cmake pkgs.spdlog pkgs.cli11 ];
          cmakeFlags = [ "-DCMAKE_PREFIX_PATH=${gz-cmake}" ];
        };

        gz-math = mkGzPkg {
          pname = "gz-math";
          version = "8.3.0";
          src = gz-math-src;
          nativeBuildInputs = [ pkgs.python3 ];
          buildInputs = [ gz-cmake gz-utils pkgs.eigen ];
          cmakeFlags = [
            "-DCMAKE_PREFIX_PATH=${gz-cmake};${gz-utils}"
          ];
        };

        sdformat = mkGzPkg {
          pname = "sdformat";
          version = "15.4.0";
          src = sdformat-src;
          nativeBuildInputs = [ pkgs.python3 ];
          buildInputs = [ gz-cmake gz-utils gz-math pkgs.tinyxml-2 pkgs.urdfdom ];
          cmakeFlags = [
            "-DCMAKE_PREFIX_PATH=${gz-cmake};${gz-utils};${gz-math}"
          ];
        };

        gz-common = mkGzPkg {
          pname = "gz-common";
          version = "6.3.0";
          src = gz-common-src;
          nativeBuildInputs = [ pkgs.python3 ];
          buildInputs = [
            gz-cmake gz-utils gz-math
            pkgs.tinyxml-2 pkgs.libuuid pkgs.gdal pkgs.assimp
            pkgs.freeimage pkgs.ffmpeg pkgs.spdlog
          ];
          cmakeFlags = [
            "-DCMAKE_PREFIX_PATH=${gz-cmake};${gz-utils};${gz-math}"
          ];
        };

        gz-plugin = mkGzPkg {
          pname = "gz-plugin";
          version = "3.1.0";
          src = gz-plugin-src;
          buildInputs = [ gz-cmake gz-utils ];
          cmakeFlags = [
            "-DCMAKE_PREFIX_PATH=${gz-cmake};${gz-utils}"
          ];
        };

        gz-msgs = mkGzPkg {
          pname = "gz-msgs";
          version = "11.1.0";
          src = gz-msgs-src;
          nativeBuildInputs = [ pkgs.protobuf pkgs.python3 ];
          buildInputs = [
            gz-cmake gz-utils gz-math
            pkgs.protobuf pkgs.tinyxml-2
          ];
          cmakeFlags = [
            "-DCMAKE_PREFIX_PATH=${gz-cmake};${gz-utils};${gz-math}"
          ];
        };

        gz-transport = mkGzPkg {
          pname = "gz-transport";
          version = "14.2.0";
          src = gz-transport-src;
          nativeBuildInputs = [ pkgs.python3 ];
          buildInputs = [
            gz-cmake gz-utils gz-math gz-msgs
            pkgs.protobuf pkgs.zeromq pkgs.cppzmq pkgs.libuuid
            pkgs.sqlite pkgs.zlib pkgs.tinyxml-2
          ] ++ transitiveDeps;
          cmakeFlags = [
            "-DCMAKE_PREFIX_PATH=${gz-cmake};${gz-utils};${gz-math};${gz-msgs}"
          ];
        };

        gz-fuel-tools = mkGzPkg {
          pname = "gz-fuel-tools";
          version = "10.1.0";
          src = gz-fuel-tools-src;
          nativeBuildInputs = [ pkgs.python3 ];
          buildInputs = [
            gz-cmake gz-utils gz-math gz-common gz-msgs gz-transport
            pkgs.curl pkgs.jsoncpp pkgs.libyaml pkgs.libzip
          ] ++ transitiveDeps;
          cmakeFlags = [
            "-DCMAKE_PREFIX_PATH=${gz-cmake};${gz-utils};${gz-math};${gz-common};${gz-msgs};${gz-transport}"
          ];
        };

        gz-rendering = mkGzPkg {
          pname = "gz-rendering";
          version = "9.5.0";
          src = gz-rendering-src;
          nativeBuildInputs = [ pkgs.python3 ];
          buildInputs = [
            gz-cmake gz-utils gz-math gz-common gz-plugin
            ogre pkgs.freeimage pkgs.xorg.libX11
            pkgs.libglvnd pkgs.mesa pkgs.eigen
            pkgs.libuuid pkgs.gdal pkgs.libGL pkgs.libGLU
            pkgs.assimp pkgs.boost
          ] ++ transitiveDeps;
          cmakeFlags = [
            "-DCMAKE_PREFIX_PATH=${gz-cmake};${gz-utils};${gz-math};${gz-common};${gz-plugin}"
          ];
          # Same patches as before:
          # (1) inject <GL/gl.h>+<GL/glext.h> before <GL/glx.h>
          # (2) replace dummyWindowId with DefaultRootWindow so OGRE skips
          #     parentWindowHandle validation (still needed against ogre1_10).
          preConfigure = ''
            for f in $(grep -rl '<GL/glx.h>' ogre/ 2>/dev/null || true); do
              sed -i 's|<GL/glx.h>|<GL/gl.h>\n# include <GL/glext.h>\n# include <GL/glx.h>|' "$f"
            done
            sed -i 's|this->CreateRenderWindow(std::to_string(this->dummyWindowId), 1, 1,|this->CreateRenderWindow(std::to_string(static_cast<unsigned long>(DefaultRootWindow(static_cast<Display*>(this->dummyDisplay)))), 1, 1,|' \
              ogre/src/OgreRenderEngine.cc 2>/dev/null || true
          '';
        };

        gz-physics = mkGzPkg {
          pname = "gz-physics";
          version = "8.3.0";
          src = gz-physics-src;
          nativeBuildInputs = [ pkgs.python3 ];
          buildInputs = [
            gz-cmake gz-utils gz-math gz-common gz-plugin
            sdformat
            pkgs.bullet pkgs.eigen pkgs.libuuid pkgs.gdal
            pkgs.tinyxml-2 pkgs.assimp
          ] ++ transitiveDeps;
          cmakeFlags = [
            "-DCMAKE_PREFIX_PATH=${gz-cmake};${gz-utils};${gz-math};${gz-common};${gz-plugin};${sdformat}"
          ];
        };

        gz-sensors = mkGzPkg {
          pname = "gz-sensors";
          version = "9.2.0";
          src = gz-sensors-src;
          nativeBuildInputs = [ pkgs.python3 ];
          buildInputs = [
            gz-cmake gz-utils gz-math gz-common gz-plugin
            gz-msgs gz-transport gz-rendering sdformat
          ] ++ transitiveDeps;
          cmakeFlags = [
            "-DCMAKE_PREFIX_PATH=${gz-cmake};${gz-utils};${gz-math};${gz-common};${gz-plugin};${gz-msgs};${gz-transport};${gz-rendering};${sdformat}"
          ];
        };

        gz-gui = mkGzPkg {
          pname = "gz-gui";
          version = "9.0.2";
          src = gz-gui-src;
          nativeBuildInputs = [ pkgs.qt5.wrapQtAppsHook pkgs.python3 ];
          buildInputs = [
            gz-cmake gz-utils gz-math gz-common gz-plugin
            gz-msgs gz-transport gz-rendering
            pkgs.qt5.qtbase pkgs.qt5.qtquickcontrols2 pkgs.qt5.qtdeclarative
          ] ++ transitiveDeps;
          cmakeFlags = [
            "-DCMAKE_PREFIX_PATH=${gz-cmake};${gz-utils};${gz-math};${gz-common};${gz-plugin};${gz-msgs};${gz-transport};${gz-rendering}"
          ];
        };

        gz-sim = mkGzPkg {
          pname = "gz-sim";
          version = "9.5.0";
          src = gz-sim-src;
          nativeBuildInputs = [ pkgs.qt5.wrapQtAppsHook pkgs.protobuf pkgs.python3 ];
          buildInputs = [
            gz-cmake gz-utils gz-math gz-common gz-plugin
            gz-msgs gz-transport gz-rendering gz-gui
            gz-physics gz-sensors sdformat gz-fuel-tools
            pkgs.qt5.qtbase pkgs.qt5.qtquickcontrols2 pkgs.qt5.qtdeclarative
            pkgs.bullet
            pkgs.ffmpeg pkgs.assimp pkgs.libyaml
          ] ++ transitiveDeps;
          cmakeFlags = [
            "-DCMAKE_PREFIX_PATH=${gz-cmake};${gz-utils};${gz-math};${gz-common};${gz-plugin};${gz-msgs};${gz-transport};${gz-rendering};${gz-gui};${gz-physics};${gz-sensors};${sdformat};${gz-fuel-tools}"
          ];
        };

        # ==================== gazebo_native bridge =================
        # Shared dimos NativeModule helpers + LCM headers live alongside
        # the hardware sensors. Pin a relative path so devShells work too.
        dimos-common = ../../../hardware/sensors/lidar/common;

        gazebo_native = pkgs.stdenv.mkDerivation {
          pname = "gazebo_native";
          version = "0.2.0";
          src = ./.;

          # Bridge is a CLI tool; Qt comes in via gz-sim's transitive deps.
          dontWrapQtApps = true;

          nativeBuildInputs = [ pkgs.cmake pkgs.pkg-config pkgs.makeWrapper ];
          buildInputs = [
            gz-cmake gz-utils gz-math gz-common gz-plugin
            gz-msgs gz-transport gz-rendering gz-physics gz-sensors
            gz-fuel-tools gz-gui gz-sim
            sdformat
            pkgs.lcm pkgs.glib pkgs.protobuf pkgs.libsodium
            pkgs.qt5.qtbase pkgs.qt5.qtquickcontrols2 pkgs.qt5.qtdeclarative
            pkgs.bullet pkgs.ffmpeg pkgs.assimp ogre
            pkgs.libyaml pkgs.urdfdom pkgs.boost
            pkgs.libGL pkgs.libGLU
          ] ++ transitiveDeps;

          cmakeFlags = [
            "-DCMAKE_POLICY_VERSION_MINIMUM=3.5"
            "-DFETCHCONTENT_SOURCE_DIR_DIMOS_LCM=${dimos-lcm}"
            "-DDIMOS_COMMON_DIR=${dimos-common}"
            "-DCMAKE_PREFIX_PATH=${gz-cmake};${gz-utils};${gz-math};${gz-common};${gz-plugin};${gz-msgs};${gz-transport};${gz-rendering};${gz-physics};${gz-sensors};${gz-fuel-tools};${gz-gui};${gz-sim};${sdformat}"
          ];

          # Tell gz-sim where its plugin .so's live, otherwise the embedded
          # Server can't load gz-sim-physics-system, sensors-system, etc.
          # GZ_RENDERING_PLUGIN_PATH points at the new ogre engine plugin.
          postInstall = ''
            wrapProgram $out/bin/gazebo_native \
              --prefix GZ_SIM_SYSTEM_PLUGIN_PATH    : ${gz-sim}/lib/gz-sim-9/plugins \
              --prefix GZ_SIM_RESOURCE_PATH         : ${gz-sim}/share/gz/gz-sim9 \
              --prefix GZ_SIM_PHYSICS_ENGINE_PATH   : ${gz-physics}/lib \
              --prefix GZ_GUI_PLUGIN_PATH           : ${gz-gui}/lib/gz-gui-9/plugins \
              --prefix GZ_RENDERING_PLUGIN_PATH     : ${gz-rendering}/lib/gz-rendering-9/engine-plugins \
              --prefix GZ_RENDERING_RESOURCE_PATH   : ${gz-rendering}/share/gz/gz-rendering9 \
              --set    GZ_CONFIG_PATH                 ${gz-cmake}/share/gz
          '';
        };

      in {
        packages = {
          default = gazebo_native;
          inherit
            gz-cmake gz-utils gz-math sdformat
            gz-common gz-plugin gz-msgs gz-transport
            gz-fuel-tools gz-rendering gz-gui
            gz-physics gz-sensors gz-sim
            gazebo_native;
        };

        devShells.default = pkgs.mkShell {
          buildInputs = [ gz-sim gazebo_native ];
        };
      });
}
