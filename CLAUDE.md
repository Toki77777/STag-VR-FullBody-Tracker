# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A desktop app that does VR full-body tracking from a single camera by detecting fiducial markers worn on the legs and hips, and feeds the resulting poses into SteamVR as trackers. This repo is a fork of `April-Tag-VR-FullBody-Tracker` (originally by ju1ce) **migrated from AprilTag/ArUco markers to the STag marker system** (ManfredStoiber/stag).

**Naming caveat:** despite the repo name, the migration kept the legacy `AprilTag` names everywhere. The app directory, executable, and CMake project are all still `AprilTagTrackers` / `April-Tag-VR-FullBody-Tracker`, driver trackers are named `ApriltagTracker0/1/2`, and there is still an `ArucoConfig`. Only marker *detection* was swapped to STag — multi-marker board pose solving still uses `cv::aruco` boards. Don't "fix" these names.

## Build & run

CMake + vcpkg (manifest mode, `vcpkg.json`). C++20. Windows is the primary platform (MSVC); Linux is supported. On Windows, build from Visual Studio or a VS Developer Command Prompt (MSVC must be on PATH).

```bash
cmake -B build
cmake --build build --config Release --target install
```

- **vcpkg auto-bootstraps.** `att_bootstrap_vcpkg()` (in `CMake/helpers.cmake`) clones vcpkg into `./vcpkg` (or uses `$VCPKG_ROOT`) on first configure and sets the toolchain. No manual vcpkg setup needed. First build is very slow (compiles OpenCV, wxWidgets, etc.). Default triplet is static (`x64-windows-static`).
- **Install goes to `./install`**, not Program Files (overridden in the top-level `CMakeLists.txt`).
- The `BridgeDriver` git submodule is auto-cloned during configure (`att_clone_submodule`).

Useful CMake options (top-level `CMakeLists.txt`):
- `-DATT_DEBUG=ON` — developer mode: custom asserts, debug logging, debugger support. Works even in a Release build.
- `-DATT_LOG_LEVEL=0|1|2` — 0 silent, 1 info (default), 2 debug.
- `-DENABLE_TESTING=ON` — build the `test` target (off by default).
- `-DATT_ENABLE_ASAN=ON`, `-DATT_ENABLE_ANALYZER=ON` (analyzer needs `ATT_DEBUG`).
- `-DATT_ENABLE_DRIVER=OFF` — skip building the SteamVR bridge driver.

### Tests

Tests use **doctest and are co-located in the source `.cpp` files** (e.g. `TEST_CASE` blocks in `tracker/VideoCapture.cpp`, `tracker/VRDriver.cpp`), compiled only when `ATT_TESTING` is defined. The `test` target (`AprilTagTrackers/test/CMakeLists.txt`) recompiles the shared `ATT_TESTABLE_SOURCES` list with a doctest runner (`test/main.cpp`) and a GUI stub. Enable with `-DENABLE_TESTING=ON`, then build and run the `test` executable; pass doctest CLI flags (e.g. `--test-case="..."`) to run a single case.

## Architecture

The pipeline is: **camera → marker detection → per-tracker board pose → playspace transform → SteamVR driver over IPC.**

- **`MyApp`** (`MyApp.cpp`, a `wxApp`) is the entry point. It owns the `UserConfig`/`ArucoConfig`/`Localization` and constructs the `Tracker` and `GUI`.
- **`Tracker`** (`Tracker.cpp/.hpp`) implements `ITrackerControl` (the interface the GUI drives). It owns the camera/main threads, `VideoCapture`, `VRDriver`, the OpenVR client, `PlayspaceCalib`, and the vector of `TrackerUnit`s. It also contains the camera- and tracker-calibration routines.
- **`tracker::MainLoopRunner`** (`tracker/MainLoopRunner.hpp`) is the heart of tracking — the per-frame loop. Read this first to understand runtime behavior. Each frame it: converts to grayscale; asks the driver for each tracker's predicted pose and **projects it to build a search mask** (a circular or full-height-strip window) to speed up detection; runs `StagWrapper::DetectMarkers`; calls `math::EstimatePoseTracker` per tracker board; applies depth smoothing and FOV/behind-camera rejection; transforms the pose through `PlayspaceCalib`; and sends it to the driver via `VRDriver::UpdateTracker`.
- **`StagWrapper`** (`StagWrapper.cpp/.hpp`) wraps the STag library. `markerLibrary` config index maps to a Hamming-distance library (HD11..HD23, see `STAG_LIBRARY_HDS`). Detection runs on a decimated image; corners are refined at full res and returned **ArUco-compatible** so the downstream `cv::aruco` board pose solving is unchanged.
- **`tracker::VRDriver`** (`tracker/VRDriver.hpp`) talks to the SteamVR bridge driver with a **text command protocol over IPC** (`updatepose`, `gettrackerpose`, `addtracker`, `updatestation`, …). IPC is a Windows named pipe (`IPC/WindowsNamedPipe.cpp`) or a UNIX socket (`IPC/UNIXSocket.cpp`), selected at build time.
- **`BridgeDriver/`** is the SteamVR OpenVR driver (submodule `Simple-OpenVR-Bridge-Driver`, branch `AprilTagTrackers`). It hosts the named pipe and creates/moves the SteamVR trackers. It must be installed into SteamVR's `drivers` folder (see `BridgeDriver/install_driver.*`, installed to `driver_files/` next to the driver).
- **`OpenVRClient`** (`tracker/OpenVRClient.cpp`) reads HMD/controller poses from SteamVR, used for playspace calibration.
- **`GUI`** (`GUI.hpp`, `GUI/MainFrame.cpp`) is wxWidgets. It is a **thread-safe facade**: calls that must run on the main thread are wrapped in `CallAfter`. The Tracker never touches wx directly — it goes through the `GUI` interface, and the GUI drives the Tracker through `ITrackerControl`.

## Config & serialization (custom, non-obvious)

Config objects use a homegrown compile-time reflection system, not a library:

- `utils/Reflectable.hpp` defines `REFLECTABLE_BEGIN` / `REFLECTABLE_FIELD(type, name)` / `REFLECTABLE_END` macros (implemented with `__COUNTER__`). The `Reflect::ForEach` helper iterates fields.
- `serial::Serializable<T>` (`serial/Serializable.hpp`) gives any reflectable config a `Save()`/`Load()` that (de)serializes to **YAML via OpenCV `FileStorage`** (`serial/FileStorage.*`, `serial/CustomSerial.*`).
- The three config classes live in `Config.hpp` and write to the config dir (`utils::GetConfigDir()`): `UserConfig` → `config.yaml`, `CalibrationConfig` → `calib.yaml`, `ArucoConfig` → `aruco.yaml`. Localization strings are also reflectable (`Localization.hpp`).
- Field types in `config/`: `cfg::Validated<T>` wraps a value with a constraint (`cfg::Clamp`, `cfg::GreaterEqual`); `cfg::List<T>` is a sized list; also `ManualCalib`, `TrackerUnit`, `VideoStream`. Use `ATT_SERIAL_COMMENT("...")` to attach a YAML comment.

**To add a user setting:** add a `REFLECTABLE_FIELD` to `UserConfig` in `Config.hpp` — it is then auto-serialized and available to the GUI form. There is no separate registration step.

## Conventions

Enforced by `.clang-format` (LLVM base, 4-space indent, no tabs, `ColumnLimit: 0`, left pointer alignment) and `.clang-tidy`. Run clang-format before committing.

- Naming: classes/functions/enums `CamelCase`; variables/params `camelBack`; private members `m`-prefixed (`mCapture`); global constants `UPPER_CASE`; macros `ATT_`-prefixed `UPPER_CASE`.
- Everything project-wide is `ATT_`-prefixed: macros, CMake vars/functions (`att_*`), compile defines (`ATT_OS_*`, `ATT_COMP_*`).
- Use `ATT_ASSERT` / `ATT_REQUIRE` (`utils/Assert.hpp`), not raw `assert`. Log with `ATT_LOG_ERROR` / `ATT_LOG_INFO` (`utils/Log.hpp`).
- Includes in `AprilTagTrackers/` are rooted at that directory (e.g. `#include "tracker/VRDriver.hpp"`), not relative.

## Markers & utilities

- `images-to-print/` — ready-to-print A4 STag (HD11) sheets, 93mm markers, sized for the default 3-tracker setup (one front + one back sheet per tracker).
- `utilities/generate_stag_markers.py` — generate custom marker sheets (needs Python with numpy, opencv-python, pillow).
- `utilities/set_exposure.bat` — camera exposure helper.
- `locales/` — translation files (en, ja, ru, zh-cn; see `Localization::LANG_CODE_MAP`).
