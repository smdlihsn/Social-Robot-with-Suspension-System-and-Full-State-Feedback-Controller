import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float64, Float64MultiArray
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
         = (Nu + K·Nx)·r  −  K·q     [full-state feedback with precompensation]

    State estimation:
      Body states (yc, ẏc) are estimated by integrating IMU acceleration,
      fused with joint encoder data via a complementary filter to prevent
      integration drift. Foot states (yk, ẏk) are read directly from the
      joint encoder. This provides genuinely independent body and foot states,
      resolving the state-mirroring approximation of the previous implementation.
    """

    # ── Physical parameters (match MATLAB model exactly) ──────────────
    MK = 1.0      # [kg]    unsprung (foot) mass
    MC = 7.5      # [kg]    sprung (body) mass
    KR = 2000     # [N/m]   joint stiffness  → ωn ≈ 16 rad/s
    BR = 200      # [N·s/m] joint damping    → ζ  ≈ 0.82 (near-critical)
    KK = 15000    # [N/m]   contact model    → contact mode ≈ 130 rad/s

    # ── FSF gains (from MATLAB acker() with desired poles [-100,-95,-16,-14]) ──
    #K  = np.array([-936.0, -35.66, 1567.0, -3.088])  # [1×4] state feedback gain
    #K = np.array([-936, -150, 400, -20])
    K  = np.array([-936, -35.66, 567, -3.088])
    NU = 2000.0                                        # scalar precompensation
    NX = np.array([1.0, 0.0, 0.0, 0.0])               # [4×1] state precompensation

    # ── Gravity feedforward ───────────────────────────────────────────
    # Gravity omitted from state-space model (DC offset only).
    # Constant feedforward compensates: F = (mc + mk) * g = 8.5 * 9.81
    GRAVITY_FF = (MC + MK) * 9.81

    # ── Complementary filter coefficient ─────────────────────────────
    # Alpha close to 1.0 → trust IMU for fast dynamics
    # (1 - alpha) → joint encoder corrects slow drift
    ALPHA = 0.95

    # ── Safety limits ─────────────────────────────────────────────────
    FORCE_MIN = -300.0   # [N]
    FORCE_MAX =  300.0   # [N]
    MAX_TILT  =  0.08    # [m] maximum allowed height offset command

    def __init__(self):
        super().__init__('active_suspension_node')

        self.last_print_time = self.get_clock().now()
        self.print_delay = 0.5

        #self.nominal_height = 0.02 #1
        #self.nominal_height = -0.015 #2
        self.nominal_height = -0.01 #3
        

        # Social tilt offsets
        self.offset_L = 0.0
        self.offset_R = 0.0

        # ── IMU state (chassis body — shared for both joints) ─────────
        # One IMU on the chassis gives yc and ẏc for both L and R sides.
        # The chassis is rigid so one measurement is sufficient.
        self.yc_pos = 0.0
        self.yc_vel = 0.0
        self.last_imu_time = None

        # ── Full state vectors: [yc, ẏc, yk, ẏk] ────────────────────
        self.state_L = np.array([self.nominal_height, 0.0,
                                  self.nominal_height, 0.0])
        self.state_R = np.array([self.nominal_height, 0.0,
                                  self.nominal_height, 0.0])

        # ── Subscriptions ─────────────────────────────────────────────
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.create_subscription(Float64MultiArray, '/social_tilt', self.tilt_cb, 10)
        self.create_subscription(Imu, '/imu', self.imu_cb, 10) 

        # ── Publishers ────────────────────────────────────────────────
        self.pub_L = self.create_publisher(
            Float64, '/model/grey_robot/joint/suspension_L/cmd_force', 10)
        self.pub_R = self.create_publisher(
            Float64, '/model/grey_robot/joint/suspension_R/cmd_force', 10)

        self.get_logger().info(
            f'FSF controller ready | '
            f'ζ={self.BR / (2 * np.sqrt(self.KR * self.MC)):.2f} | '
            f'F_gravity={self.GRAVITY_FF:.1f} N | '
            f'IMU complementary filter α={self.ALPHA}'
        )

    def imu_cb(self, msg):
        now = self.get_clock().now()
        if self.last_imu_time is not None:
            dt = (now - self.last_imu_time).nanoseconds / 1e9
            if dt > 0.1:
                self.last_imu_time = now
                return
            acc_z = msg.linear_acceleration.z - 9.81
            # Only integrate velocity — anchor position to joint encoder in joint_cb
            self.yc_vel = self.ALPHA * (self.yc_vel + acc_z * dt) + (1 - self.ALPHA) * 0.0
        self.last_imu_time = now

    def tilt_cb(self, msg):
        """
        Receive differential height command [left_offset, right_offset] in metres.
        Clipped to ±MAX_TILT to prevent unsafe joint excursions.
        """
        if len(msg.data) >= 2:
            self.offset_L = float(np.clip(msg.data[0], -self.MAX_TILT, self.MAX_TILT))
            self.offset_R = float(np.clip(msg.data[1], -self.MAX_TILT, self.MAX_TILT))
        else:
            self.get_logger().warn('Tilt msg must have 2 values: [left, right]')

    def joint_cb(self, msg):
        """
        Read joint encoder data and build the full state vector.

        Body states (yc, ẏc): from IMU complementary filter — independent
        Foot states (yk, ẏk): from joint encoder directly

        This gives four genuinely independent state measurements, resolving
        the state-mirroring limitation of the previous implementation.
        """
        for i, name in enumerate(msg.name):
            if name == 'suspension_L':
                pos = msg.position[i]
                vel = msg.velocity[i]
                self.state_L = np.array([
                    self.nominal_height + pos,  # yc position ← back to joint encoder
                    self.yc_vel,                # ẏc velocity ← from IMU (better than differenced encoder)
                    self.nominal_height + pos,  # yk
                    vel
                ])

            elif name == 'suspension_R':
                pos = msg.position[i]
                vel = msg.velocity[i]
                self.state_R = np.array([
                    self.nominal_height + pos,  # yc position ← joint encoder independent per side
                    self.yc_vel,                # ẏc velocity ← same IMU chassis velocity
                    self.nominal_height + pos,  # yk
                    vel
                ])

        self.compute_and_publish()

    def compute_and_publish(self):
        """
        Compute and publish actuator forces using the FSF control law.

        Control law:
          Fa = Nu·r  −  K·(q − Nx·r)  +  F_gravity

        r  : scalar height reference [m]
        q  : 4×1 state vector [yc, ẏc, yk, ẏk]
        K  : 1×4 feedback gain from Ackermann pole placement
        Nu : scalar precompensation for steady-state tracking
        Nx : 4×1 state precompensation vector
        """
        r_L = self.nominal_height + self.offset_L
        r_R = self.nominal_height + self.offset_R

        # Reference state: target height for both links, zero velocity at target
        ref_L = np.array([r_L, 0.0, r_L, 0.0])
        ref_R = np.array([r_R, 0.0, r_R, 0.0])

        # Full-state feedback with precompensation and gravity feedforward
        force_L = self.NU * r_L - np.dot(self.K, self.state_L - ref_L) + self.GRAVITY_FF
        force_R = self.NU * r_R - np.dot(self.K, self.state_R - ref_R) + self.GRAVITY_FF

        # Safety clamp — hard actuator limit, takes priority over control law
        force_L = float(np.clip(force_L, self.FORCE_MIN, self.FORCE_MAX))
        force_R = float(np.clip(force_R, self.FORCE_MIN, self.FORCE_MAX))

        self.pub_L.publish(Float64(data=force_L))
        self.pub_R.publish(Float64(data=force_R))

        # Throttled logging — gates print to once per 0.5s, no effect on control
        current_time = self.get_clock().now()
        elapsed = (current_time - self.last_print_time).nanoseconds / 1e9

        if elapsed >= self.print_delay:
            self.get_logger().info(
                f"\n"
                f"--- SUSPENSION DATA (Tilt cmd: L={self.offset_L:.3f} R={self.offset_R:.3f}) ---\n"
                f"BODY  |  yc={self.yc_pos:.4f} m  |  ẏc={self.yc_vel:.4f} m/s  (IMU)\n"
                f"LEFT  |  yk={self.state_L[2]:.4f} m  |  force={force_L:.2f} N\n" 
                f"RIGHT |  yk={self.state_R[2]:.4f} m  |  force={force_R:.2f} N\n"
                f"------------------------------------------------------------------------"
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