# i16 GUI preview assets

Copied from the user-provided `aubo_develop_student/aubo_i16_pc_ws/src/aubo_description`.
This is provenance information; the original student workspace is not required to run the integrated GUI.
The source package identifies Allen Liu <liug@our-robotics.com> as author and declares BSD licensing.
The package was renamed to avoid conflicting with the official `aubo_description` package.

Mesh geometry and joint origins retain the student i16 appearance. The current robot is i16H;
J3 (`foreArm_joint`) limits were corrected to ±161°. This is not a replacement with official i16H CAD.
macOS uses GLB preview meshes; Ubuntu uses DAE assets.

The GUI loads the arm from this package and the gripper/camera geometry from
`seer_description/urdf/composite_robot_stick_mono.urdf`. It obtains the camera mounting
transform from the running robot's `/robot_description`, with the local URDF as its initial fallback.
The validated camera mount is relative to `wrist3_Link`: translation `(0, 0.1, 0)` m,
rotation `(0, 0, pi)` rad. Local model formats and the USD generator now use that same mount.

The full mobile manipulator model remains in `seer_description`.
See [GUI startup and operation](../aubo_control_gui/README.md) and
[camera model details](../seer_description/README_MONO_CAMERA.md).
