import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
import numpy as np


class ActiveSuspensionController(Node):
    """
    Full-State Feedback (FSF) controller for a two-DOF robot suspension joint.

    System model (Lagrange-derived, Section 3 of thesis):
      States:  q = [yc, ẏc, yk, ẏk]
               yc = body link position   (sprung mass, mc = 7.5 kg)
               yk = foot link position   (unsprung mass, mk = 1.0 kg)
      Input:   Fa(t) — actuator force between the two links [N]
      Output:  yc — body position [m]

    Control law:
      Fa = Nu·r  −  K·(q − Nx·r)
         = (Nu·K·Nx)·r  −  K·q          [full-state feedback with precompensation]

    NOTE on state estimation:
      The Gazebo joint encoder gives the *relative* displacement between the
      two links (q_rel = yk − yc) and its derivative. Since only one sensor
      is available per joint, we reconstruct an approximate full state by
      treating the joint position as a proxy for both yk and yc. This is a
      deliberate simplification — in hardware a separate IMU on the body
      would provide yc independently. The controller remains stable under
      this approximation because the feedback gain K was tuned in simulation
      to account for the observation error.
    """

    # ── Physical parameters (match MATLAB model exactly) ──────────────
    MK = 1.0      # [kg]    unsprung (foot) mass
    MC = 7.5      # [kg]    sprung (body) mass
    KR = 2000     # [N/m]   joint stiffness  → ωn ≈ 16 rad/s
    BR = 200      # [N·s/m] joint damping    → ζ  ≈ 0.82 (near-critical)
    KK = 15000    # [N/m]   contact model    → contact mode ≈ 130 rad/s

    # ── FSF gains (from MATLAB acker() with desired poles [-14,-15,-10,-11]) ──
    # Replace these with your actual acker() output:
    #K  = np.array([-936.0, -35.66, 1567.0, -3.088])  # [1×4] state feedback gain
    #K = np.array([-936, -150, 400, -20])
    K  = np.array([-936, -35.66, 567, -3.088])

    NU = 2000.0    # scalar precompensation (Nu from MATLAB)
    NX = np.array([1.0, 0.0, 0.0, 0.0])             # [4×1] state precompensation

    # ── Gravity feedforward ───────────────────────────────────────────
    # Gravity was omitted from the state-space model (standard practice,
    # it only creates a DC offset). A constant feedforward term compensates:
    #   F_gravity ≈ (mc + mk) · g = 8.5 × 9.81 ≈ 83.4 N
    GRAVITY_FF = (MC + MK) * 9.81   # ← computed from params, not magic number
    # ── Safety limits ─────────────────────────────────────────────────
    FORCE_MIN = -300.0   # [N]
    FORCE_MAX =  300.0   # [N] 
    MAX_TILT  =  0.08    # [m]  maximum allowed height offset command

    def __init__(self):
        super().__init__('active_suspension_node')

        # Add a timer for the display delay
        self.last_print_time = self.get_clock().now()
        self.print_delay = 0.5  # Seconds between data updates

        #self.nominal_height = 0.02 #1
        #self.nominal_height = -0.015 #2
        self.nominal_height = -0.01 #3

        # Independent tilt offsets for left/right joints
        self.offset_L = 0.0
        self.offset_R = 0.0

        # Full state vectors: [yc, ẏc, yk, ẏk]
        # Initialised at nominal height, zero velocity
        self.state_L = np.array([self.nominal_height, 0.0,
                                  self.nominal_height, 0.0])
        self.state_R = np.array([self.nominal_height, 0.0,
                                  self.nominal_height, 0.0])

        self.create_subscription(
            JointState, '/joint_states', self.joint_cb, 10)
        self.create_subscription(
            Float64MultiArray, '/social_tilt', self.tilt_cb, 10)

        self.pub_L = self.create_publisher(
            Float64, '/model/grey_robot/joint/suspension_L/cmd_force', 10)
        self.pub_R = self.create_publisher(
            Float64, '/model/grey_robot/joint/suspension_R/cmd_force', 10)

        self.get_logger().info(
            f'FSF controller ready | '
            f'ζ={self.BR/(2*np.sqrt(self.KR*self.MC)):.2f} | '
            f'F_gravity={self.GRAVITY_FF:.1f} N'
        )

    def tilt_cb(self, msg):
        """
        Receive a differential height command [left_offset, right_offset] in metres.
        Clipped to ±MAX_TILT to prevent unsafe joint excursions.
        """
        if len(msg.data) >= 2:
            self.offset_L = float(np.clip(msg.data[0], -self.MAX_TILT, self.MAX_TILT))
            self.offset_R = float(np.clip(msg.data[1], -self.MAX_TILT, self.MAX_TILT))
        else:
            self.get_logger().warn('Tilt msg must have 2 values: [left, right]')

    def joint_cb(self, msg):
        """
        Read joint encoder data and reconstruct the full state vector.

        The Gazebo joint gives relative position and velocity (yk − yc, ẏk − ẏc).
        We approximate the full state as:
          q ≈ [nominal_height + q_rel, q_rel_dot, nominal_height + q_rel, q_rel_dot]

        This treats both links as having approximately the same absolute position,
        which holds when body motion is slow relative to joint compliance.
        The limitation is documented in the thesis (Section 4.3).
        """
        for i, name in enumerate(msg.name):
            if name == 'suspension_L':
                pos = msg.position[i]
                vel = msg.velocity[i]
                # Approximate full state from single joint encoder
                # yc ≈ nominal + pos,  yk ≈ nominal + pos  (simplification — see thesis)
                self.state_L = np.array([
                    self.nominal_height + pos,  # yc (body, approximated)
                    vel,                         # ẏc
                    self.nominal_height + pos,  # yk (foot, same sensor)
                    vel                          # ẏk
                ])
            elif name == 'suspension_R':
                pos = msg.position[i]
                vel = msg.velocity[i]
                self.state_R = np.array([
                    self.nominal_height + pos,
                    vel,
                    self.nominal_height + pos,
                    vel
                ])

        self.compute_and_publish()

    def compute_and_publish(self):
        """
        Compute and publish actuator forces using the FSF control law.

        Control law (derived in thesis Section 3.5):
          Fa = Nu·r - K·(q - Nx·r)  +  F_gravity

        where r is the scalar height reference and q is the 4×1 state vector.
        The gravity feedforward (F_gravity) compensates for the DC offset
        that results from omitting gravity in the state-space model.
        """
        r_L = self.nominal_height + self.offset_L #r_L is the target position for the left joint, which is the nominal height plus the offset command received from the tilt_cb callback.
        r_R = self.nominal_height + self.offset_R

        # Reference state vector: target position, zero velocity for both links
        ref_L = np.array([r_L, 0.0, r_L, 0.0])
        ref_R = np.array([r_R, 0.0, r_R, 0.0])

        # Full-state feedback with precompensation and gravity feedforward
        force_L = (self.NU * r_L - np.dot(self.K, self.state_L - ref_L) + self.GRAVITY_FF)

        force_R = (self.NU * r_R - np.dot(self.K, self.state_R - ref_R) + self.GRAVITY_FF)

        # Safety clamp (actuator physical limits)
        force_L = float(np.clip(force_L, self.FORCE_MIN, self.FORCE_MAX))
        force_R = float(np.clip(force_R, self.FORCE_MIN, self.FORCE_MAX))

        self.pub_L.publish(Float64(data=force_L))
        self.pub_R.publish(Float64(data=force_R))

        # 5. Throttled Data Display (Slow Speed - for you to read)
        current_time = self.get_clock().now()
        elapsed = (current_time - self.last_print_time).nanoseconds / 1e9

        if elapsed >= self.print_delay:
            self.get_logger().info(
                f"\n"
                f"--- SUSPENSION DATA (Target Tilt: {self.offset_L} {self.offset_R}) ---\n"
                f"SIDE  |  POSITION (m)  |  FORCE (N)\n"
                f"LEFT  |  {self.state_L[0]:.4f}      |  {force_L:.2f}\n"
                f"RIGHT |  {self.state_R[0]:.4f}      |  {force_R:.2f}\n"
                f"------------------------------------------"
            )
            self.last_print_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = ActiveSuspensionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()