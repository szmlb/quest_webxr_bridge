# Coordinate frames and velocities

## Coordinate conversion

WebXR: +X right, +Y up, -Z forward. ROS: +X forward, +Y left, +Z up.

Positions (and linear and angular velocities) are rotated by a constant matrix $`C`$:

```math
\mathbf{p}_R = C\,\mathbf{p}_W, \qquad C = \begin{bmatrix} 0 & 0 & -1 \\ -1 & 0 & 0 \\ 0 & 1 & 0 \end{bmatrix}, \qquad \det C = 1
```

that is, $`(x, y, z)_R = (-z, -x, y)_W`$.

Orientations are conjugated by the same matrix, which for quaternions reduces to
a component permutation:

```math
R_R = C\,R_W\,C^\top \quad\Longleftrightarrow\quad (q_x, q_y, q_z, q_w)_R = (-q_z, -q_x, q_y, q_w)_W
```

The conversion is implemented only in the browser (`web/js/core.js`).
`test/test_frames.py` verifies numerically that the quaternion remapping equals
$`C R C^\top`$ (200 random rotations) and that the JavaScript implementation gives
the same numbers as the Python reference.

The reference space is `local-floor` (fallback `local`): its origin is on the
floor where the session started and its forward direction is where the
headset faced at that moment. ROS +X therefore points to where you looked when
pressing "Enter VR". Because Quest2ROS2 uses relative motion, only the axis
directions matter: face the robot's forward (+X) direction when entering VR, or
re-anchor after turning (lower button twice). The output `frame_id` is `world`
as in `SimulationInput.py`.

The Quest2ROS app itself is closed source, so its exact frame convention could
not be inspected. This bridge follows the ROS convention implied by
`SimulationInput.py` and by how the controllers add the offset in the robot
base frame.

## Velocities

`XRPose.linearVelocity` / `angularVelocity` are used when the browser provides
them. Otherwise velocities are estimated by finite differences with a
first-order low-pass filter (time constant 0.05 s); the estimate is reset after
tracking gaps longer than 0.25 s.
