# PX4 Patch Status

Reference PX4 checkout:

- Repository: `PX4/PX4-Autopilot`
- Branch: `main`
- Base commit observed during the audit: `f38aba3c5be9`
- Gazebo-models submodule commit: `fe3fe236e36a`

The local PX4 working tree contains project-specific changes in:

- `ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500`
- `ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt`
- `src/modules/simulation/gz_bridge/CMakeLists.txt`
- `src/modules/simulation/gz_bridge/GZBridge.cpp`
- `src/modules/simulation/gz_bridge/GZBridge.hpp`
- `src/modules/simulation/gz_bridge/server.config`
- `src/modules/simulation/gz_plugins/gstreamer/GstCameraSystem.cpp`
- `src/modules/simulation/gz_plugins/gstreamer/GstCameraSystem.hpp`
- `src/modules/simulation/gz_plugins/gstreamer/README.md`

The Gazebo-models submodule also contains changes to:

- `server.config`
- `worlds/baylands.sdf`
- Project-specific models and world assets copied into this repository.

The PX4 checkout contains many unrelated generated/untracked `Makefile`
artifacts. Those are not considered project source and are intentionally not
copied.

## Publication gate

Before claiming clean-checkout reproducibility:

1. Separate functional PX4/Gazebo changes from generated files.
2. Review each functional diff.
3. Preserve upstream copyright and license notices.
4. Create a minimal patch series against the documented PX4 base commit.
5. Apply the patches to a fresh PX4 clone and rebuild SITL.
6. Re-run the three-UAV simulation from the documented commands.

No PX4 history rewrite or modification of the original local checkout is part
of this portfolio preparation.
