# Simulation Assets

This directory packages the project-specific simulation assets that were
previously stored only in the local PX4 checkout.

## Included

- PX4 SITL airframe `4022_gz_x500_down_cam`.
- Gazebo model `x500_down_cam`.
- Downward camera model.
- Red and blue field targets.
- Six QR target models.
- Modified Baylands world files.
- The upstream PX4 Gazebo-models BSD 3-Clause license.

## Intended PX4 locations

Copy the airframe file to:

```text
PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/
```

Copy the model directories to:

```text
PX4-Autopilot/Tools/simulation/gz/models/
```

Copy the world files to:

```text
PX4-Autopilot/Tools/simulation/gz/worlds/
```

The PX4 airframe list must also include:

```cmake
4022_gz_x500_down_cam
```

Do not overwrite an existing PX4 checkout without reviewing local changes
first.

## Remaining PX4 changes

The local reference checkout contains additional source changes that are not
yet packaged here. See [PX4 Patch Status](px4-patch-status.md).

Until those changes are reviewed and represented as patches, these assets
improve traceability but do not make the complete simulation reproducible on a
clean PX4 checkout.

## Licensing

The copied PX4 Gazebo-model assets are kept with the upstream BSD 3-Clause
license in `px4-gazebo-models/LICENSE`.

The license for the Orion ROS 2 workspace itself is a separate decision and
must be selected before public release.
