# RX1 ROS 2 TODO

- Confirm the RViz orange goal-state overlay behaves correctly after selecting each MoveIt planning group.
- Re-test all MoveIt groups in fresh sessions: `right_arm`, `left_arm`, `head`, `torso`, `right_arm_with_torso`, `left_arm_with_torso`, and `dual_arms`.
- Polish the fake hardware path so repeated `Plan and Execute` runs stay reliable across all supported groups.
- Decide the next execution target: keep RViz fake hardware only, add Gazebo execution, or wire real hardware.
- If simulation execution is needed, connect MoveIt to a proper ROS 2 controller stack for RX1.
- Simplify collision geometry in the URDF so MoveIt stops warning about high-vertex collision meshes.
- Add a dummy root link above `base_link` to remove the KDL root-inertia warning.
- Either configure an octomap sensor source or disable the unused occupancy-map path in the MoveIt demo.
- Do a systematic self-collision review and tighten the SRDF collision matrix where needed.
- Update the README with the current ROS 2 launch commands, fake-hardware workflow, and known limitations.
- After the sim path is stable, decide whether to connect MoveIt trajectory execution to `rx1_motor` and `feetech_lib` for the real robot.
