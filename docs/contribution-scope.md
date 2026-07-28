# Contribution Scope

This document separates individual work from the integrated six-member Orion
Team project.

## Hüseyin Sefa Kiriş

Role: Orion Team founder and captain; software architecture and autonomy.

Verified contribution scope:

- Defined the overall software architecture.
- Selected and introduced ROS 2 as the internal integration layer.
- Designed the node layout, topic structure, and JSON message contracts.
- Developed `iha_pkg` from the beginning, including:
  - MAVSDK/PX4 connection handling.
  - Asynchronous telemetry collection.
  - Leader and follower state machines.
  - Dynamic role and slot configuration.
  - Formation geometry and transitions.
  - Offboard velocity control and navigation helpers.
  - Swarm telemetry and mission-state synchronization.
  - QR mission handoff integration.
  - Follower separation and return-to-home flows.
- Built the PX4 SITL/Gazebo simulation setup and simulation scenarios.
- Led and coordinated the software work of the six-person team.

## Saadet Bayrakol

Role: Image processing.

Verified contribution scope:

- Developed the image-processing implementation in `g_isleme_pkg`.

GitHub: [bayrakolsaadet](https://github.com/bayrakolsaadet)

## Eda Lazoğlu

Role: Ground-control interface.

Verified contribution scope:

- Developed the user-interface implementation in `arayuz_pkg`.

GitHub: [EdanurLazoglu](https://github.com/EdanurLazoglu)

## Integration boundary

Hüseyin Sefa Kiriş designed the system-level communication and integration in
which these components operate, but does not present Saadet Bayrakol's or Eda
Lazoğlu's implementation as his individual code.

## Why all packages are kept together

The project is a distributed ROS 2 system. Publishing only one package would
hide the message contracts, runtime integration, and end-to-end mission flow.
The integrated workspace is therefore retained while the ownership boundary is
documented explicitly.

## Claims intentionally not made

This repository does not claim that:

- Every line was written by one person.
- A successful build proves real-flight safety.
- Simulation evidence is equivalent to hardware flight evidence.
- The code is production-ready or certified flight software.

Before any public release, the team-code publication permission and repository
license must be decided explicitly.
