#!/usr/bin/env python
"""
MPC-based trajectory tracker for an Ackermann mobile robot with time compliance,
smooth control, proper termination, current reference pose visualization,
delay compensation, velocity scaling, and an additional cost for lateral error (staying on track).

This node subscribes to:
  - /mushr2/car_pose (geometry_msgs/PoseStamped)
  - /mushr2/relopush/serialized_trajectory (std_msgs/String)

It publishes:
  - /mushr2/mux/ackermann_cmd_mux/input/navigation (AckermannDriveStamped)
  - /mushr2/nav_path_viz (nav_msgs/Path) for trajectory visualization
  - /mushr2/ref_pose_markers (visualization_msgs/MarkerArray) for complete trajectory visualization in RViz
  - /mushr2/tracked_ref_pose (geometry_msgs/PoseStamped) for the current reference pose

Each trajectory waypoint must have: x, y, yaw, time, and ref_vel.
The 'time' field is relative to the trajectory start.
Delay compensation is added to account for actuation latency.
The computed velocity is scaled by a factor `vel_scale` for forward motion and by
a different factor `vel_scale_back` when going backward.
An additional cost term penalizes the lateral error (y error in the robot frame) to help the robot stay on track.
"""

import rospy
import numpy as np
import base64
import math
from scipy.optimize import minimize

from geometry_msgs.msg import PoseStamped, Quaternion
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Path
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

# Import your ReloPushTrajectory module and your utility module (renamed to tf_utils).
import ReloPushTrajectory
import tf_utils  # should provide angle_to_rosquaternion(yaw)

class MPCControllerNode(object):
    def __init__(self):
        rospy.init_node('mpc_controller_node', anonymous=True)

        # --- Robot and MPC parameters ---
        self.L = rospy.get_param("~wheelbase", 0.29)          # Wheelbase (meters)
        self.dt = rospy.get_param("~dt", 0.05)                  # Time step [s]
        self.horizon = rospy.get_param("~horizon", 8)          # Prediction horizon (steps)
        # Cost weights (tune these as needed)
        self.w_dist    = rospy.get_param("~w_dist", 5.0)      # Weight on squared position error #50
        self.w_yaw     = rospy.get_param("~w_yaw", 1.0)        # Weight on squared heading error #0.6
        self.w_vel     = rospy.get_param("~w_vel", 0.2)        # Weight on squared velocity error #0.1
        self.w_control = rospy.get_param("~w_control", 0.001)    # Weight on control effort
        self.w_delta_rate = rospy.get_param("~w_delta_rate", 50.0)  # Weight on rapid steering rate changes #90
        # New cost weight for lateral error (error along robot's y-axis)
        self.w_lat = rospy.get_param("~w_lat", 40.0)   #1.0

        self.max_v     = rospy.get_param("~max_v", 0.45)        # Maximum absolute linear velocity

        self.max_v_push = rospy.get_param("~max_v_push", 0.295) #0.32
        self.max_v_nonpush = rospy.get_param("~max_v_nonpush", 0.38)

        self.max_steer = rospy.get_param("~max_steer", 0.33)      # Maximum absolute steering angle (rad)

        # --- Acceleration limit parameter ---
        self.max_accel = rospy.get_param("~max_accel", 0.73)   #0.73  # Maximum acceleration/deceleration (m/s²)

        # --- Delay compensation parameter ---
        self.control_delay = rospy.get_param("~control_delay", 0.05)

        # --- Velocity scaling parameters ---
        # Scale for forward velocities.
        self.vel_scale = rospy.get_param("~vel_scale", 1.01) #93 #0.945
        # Scale for backward velocities (if negative, we apply a higher scaling factor).
        self.vel_scale_back = rospy.get_param("~vel_scale_back", 0.9) #0.8

        # --- State and reference trajectory ---
        self.current_pose = None         # Latest pose from /mushr2/car_pose
        self.current_yaw  = 0.0           # Latest heading (radians)
        # Reference trajectory stored as a NumPy array with shape (N, 5):
        # [x, y, yaw, time, ref_vel]
        self.ref_traj = None
        self.path_received = False

        self.is_pushing = False

        self.traj_start_time = None      # Time when trajectory is received

        self.prev_steer = 0.0            # For computing steering_angle_velocity

        self.last_cmd = np.array([0.0, 0.0])  # Last commanded control [v, delta]

        # --- Publisher for current tracked reference pose ---
        self.tracked_ref_pub = rospy.Publisher("/mushr2/tracked_ref_pose", 
                                               PoseStamped, queue_size=10)

        # --- Subscribers ---
        rospy.Subscriber("/mushr2/car_pose", PoseStamped, self.cb_pose)
        rospy.Subscriber("/mushr2/relopush/serialized_trajectory", String, self.cb_traj)

        # --- Other Publishers ---
        self.ack_pub = rospy.Publisher("/mushr2/mux/ackermann_cmd_mux/input/navigation", 
                                       AckermannDriveStamped, queue_size=10)
        self.nav_path_viz = rospy.Publisher("/mushr2/nav_path_viz", Path, queue_size=10)
        self.ref_pose_marker_pub = rospy.Publisher("/mushr2/ref_pose_markers", MarkerArray, queue_size=10)

        rospy.Timer(rospy.Duration(self.dt), self.control_loop)
        rospy.loginfo("MPC Controller Node with delay compensation, velocity scaling, and lateral error cost started.")

    def cb_pose(self, msg):
        """Update the current robot pose."""
        self.current_pose = msg.pose
        q = msg.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny, cosy)

    def cb_traj(self, msg):
        """
        Decode the Base64-encoded reference trajectory.
        Each waypoint must have: x, y, yaw, time, and ref_vel.
        The 'time' field is relative to the trajectory start.
        """
        rospy.loginfo("Reference trajectory received.")
        encoded_str = msg.data
        try:
            binary_data = base64.b64decode(encoded_str)
            traj_in = ReloPushTrajectory.ReloPush_trajectory(binary_data)
            traj_in.print()  # Debug: print details

            self.traj_start_time = rospy.Time.now()
            traj_list = []
            for p in traj_in.trajectory_points:
                traj_list.append([p.x, p.y, p.yaw, p.time, p.ref_vel, p.is_pushing])
            self.ref_traj = np.array(traj_list)
            self.path_received = True

            # Visualize the full trajectory.
            path_msg = self.traj_to_nav_path(traj_in)
            self.nav_path_viz.publish(path_msg)
            self.publish_ref_pose_markers(traj_in)

            rospy.loginfo("Trajectory set and visualization published.")
        except Exception as e:
            rospy.logerr("Failed to decode reference trajectory: %s", e)

    def traj_to_nav_path(self, traj_in):
        """Convert the trajectory to a nav_msgs/Path message for visualization."""
        path_msg = Path()
        time_ref = rospy.Time.now()
        path_msg.header.stamp = time_ref
        path_msg.header.frame_id = 'map'
        for p in traj_in.trajectory_points:
            ps = PoseStamped()
            ps.pose.position.x = p.x
            ps.pose.position.y = p.y
            ps.pose.orientation = tf_utils.angle_to_rosquaternion(p.yaw)
            ps.header.stamp = self.traj_start_time + rospy.Duration(p.time)
            path_msg.poses.append(ps)
        return path_msg

    def publish_ref_pose_markers(self, traj_in):
        """Publish reference trajectory poses as a MarkerArray for RViz visualization."""
        marker_array = MarkerArray()
        for idx, p in enumerate(traj_in.trajectory_points):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "ref_pose"
            marker.id = idx
            marker.type = Marker.ARROW
            marker.action = Marker.ADD

            marker.pose.position.x = p.x
            marker.pose.position.y = p.y
            marker.pose.position.z = 0.0
            marker.pose.orientation = tf_utils.angle_to_rosquaternion(p.yaw)

            marker.scale.x = 0.5  # Arrow length
            marker.scale.y = 0.1  # Shaft diameter
            marker.scale.z = 0.1  # Head diameter

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            marker.lifetime = rospy.Duration(0)
            marker_array.markers.append(marker)
        self.ref_pose_marker_pub.publish(marker_array)

    def get_ref_state_at_time(self, abs_time):
        """
        Interpolate the reference state for a given absolute time.
        Returns [x, y, yaw, ref_vel] using linear interpolation.
        If abs_time is before the first waypoint, returns the first;
        if after the last, returns the last.
        """
        traj_abs = self.traj_start_time.to_sec() + self.ref_traj[:, 3]
        if abs_time <= traj_abs[0]:
            return self.ref_traj[0, :4]
        if abs_time >= traj_abs[-1]:
            return self.ref_traj[-1, :4]

        i = np.searchsorted(traj_abs, abs_time) - 1
        i = int(np.clip(i, 0, len(traj_abs) - 2))
        t0, t1 = traj_abs[i], traj_abs[i+1]
        f = (abs_time - t0) / (t1 - t0)
        x = (1 - f) * self.ref_traj[i, 0] + f * self.ref_traj[i+1, 0]
        y = (1 - f) * self.ref_traj[i, 1] + f * self.ref_traj[i+1, 1]
        ref_vel = (1 - f) * self.ref_traj[i, 4] + f * self.ref_traj[i+1, 4]
        yaw0 = self.ref_traj[i, 2]
        yaw1 = self.ref_traj[i+1, 2]
        dyaw = math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0))
        yaw = yaw0 + f * dyaw
        return np.array([x, y, yaw, ref_vel]), self.ref_traj[i+1,5] # is_pushing

    def predict_state(self, state, last_cmd, delay):
        """
        Predict the state after a given delay period, assuming the last command [v, delta]
        is applied continuously during that delay.
        """
        steps = int(delay / self.dt)
        s = state.copy()
        v, delta = last_cmd
        for _ in range(steps):
            s[0] += v * math.cos(s[2]) * self.dt
            s[1] += v * math.sin(s[2]) * self.dt
            s[2] += (v / self.L) * math.tan(delta) * self.dt
        return s

    def control_loop(self, event):
        """
        Main control loop:
         - If the trajectory is complete, command a stop.
         - Otherwise, use delay compensation to predict the future state,
           solve the MPC optimization, and publish the control command.
         - Also, publish the current reference pose as a PoseStamped.
        """
        if self.current_pose is None or not self.path_received:
            return

        #for debug
        #print(self.max_v)

        current_time = rospy.Time.now().to_sec()
        traj_end_time = self.traj_start_time.to_sec() + self.ref_traj[-2, 3] + 0.3
        finish_waiting = 1.0
        if current_time > traj_end_time + finish_waiting:
            cmd_msg = AckermannDriveStamped()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.drive.speed = 0.0
            cmd_msg.drive.steering_angle = 0.0
            cmd_msg.drive.steering_angle_velocity = 0.0
            self.ack_pub.publish(cmd_msg)
            rospy.loginfo("Trajectory complete. Stopping vehicle.")
            return

        # Get current measured state.
        state = np.array([self.current_pose.position.x,
                          self.current_pose.position.y,
                          self.current_yaw])
        # Predict future state using delay compensation.
        pred_state = self.predict_state(state, self.last_cmd, self.control_delay)
        # Solve MPC optimization using the predicted state.
        u_opt = self.mpc_control(pred_state)

        # Apply acceleration limit to the velocity (post-optimization).
        v_prev = self.last_cmd[0]
        new_v = u_opt[0]
        max_delta_v = self.max_accel * self.dt
        if new_v > v_prev + max_delta_v:
            new_v = v_prev + max_delta_v
        elif new_v < v_prev - max_delta_v:
            new_v = v_prev - max_delta_v
        u_opt[0] = new_v

        steering_velocity = (u_opt[1] - self.prev_steer) / self.dt
        self.prev_steer = u_opt[1]

        cmd_msg = AckermannDriveStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        # Apply different scaling factors for forward and backward directions.
        if u_opt[0] < 0:
            scaled_speed = self.vel_scale_back * u_opt[0]
        else:
            scaled_speed = self.vel_scale * u_opt[0]

        if(self.change_dir==True):
            scaled_speed*=0.77


        cmd_msg.drive.speed = float(scaled_speed)
        cmd_msg.drive.steering_angle = float(u_opt[1])
        cmd_msg.drive.steering_angle_velocity = float(steering_velocity)
        self.ack_pub.publish(cmd_msg)
        self.last_cmd = u_opt

        # Publish the currently tracked reference pose.
        ref_state, is_pushing_next = self.get_ref_state_at_time(current_time)
        ref_pose = PoseStamped()
        ref_pose.header.stamp = rospy.Time.now()
        ref_pose.header.frame_id = "map"
        ref_pose.pose.position.x = ref_state[0]
        ref_pose.pose.position.y = ref_state[1]
        ref_pose.pose.position.z = 0.0
        ref_pose.pose.orientation = tf_utils.angle_to_rosquaternion(ref_state[2])
        self.tracked_ref_pub.publish(ref_pose)

    def mpc_control(self, state):
        """
        Solve the MPC optimization over a horizon T.
        Decision variables: u = [v0, delta0, ..., v_{T-1}, delta_{T-1}].
        The cost function penalizes errors in position, heading, and velocity (w.r.t. the
        interpolated reference), the control effort, rapid changes in steering, and lateral error in the robot frame.
        """
        T = self.horizon
        u0 = np.zeros(2 * T)
        bounds = []
        for _ in range(T):
            bounds.append((-self.max_v, self.max_v))
            bounds.append((-self.max_steer, self.max_steer))
        current_time = rospy.Time.now().to_sec()

        # Check for direction change and adjust w_dist temporarily
        target_time = current_time + self.dt
        ref_state, _ = self.get_ref_state_at_time(target_time)
        ref_vel = ref_state[3]
        last_v = self.last_cmd[0]
        self.change_dir = False
        if (last_v > 0 and ref_vel < 0) or (last_v < 0 and ref_vel > 0):
            temp_w_dist = 50.0
            if(last_v > 0 and ref_vel < 0):
                self.change_dir = True
            #print("distance heavy")
        else:
            temp_w_dist = self.w_dist
            self.change_dir = False
            #print("normal weight")

        def simulate_states(u_vec):
            u_seq = u_vec.reshape((T, 2))
            s = state.copy()
            states = []
            for t in range(T):
                v = u_seq[t, 0]
                delta = u_seq[t, 1]
                x_next = s[0] + v * math.cos(s[2]) * self.dt
                y_next = s[1] + v * math.sin(s[2]) * self.dt
                theta_next = s[2] + (v / self.L) * math.tan(delta) * self.dt
                s = np.array([x_next, y_next, theta_next])
                states.append(s)
            return np.array(states)

        def objective(u_vec):
            u_seq = u_vec.reshape((T, 2))
            states = simulate_states(u_vec)
            cost = 0.0
            for t in range(T):
                target_time = current_time + (t + 1) * self.dt
                ref_state, is_pushing_next = self.get_ref_state_at_time(target_time)
                self.last_is_pushing = is_pushing_next
                x_ref, y_ref, yaw_ref, ref_vel = ref_state
                x, y, theta = states[t]
                # Compute Euclidean position error.
                pos_error = np.linalg.norm(np.array([x, y]) - np.array([x_ref, y_ref]))
                # Compute heading error.
                heading_error = math.atan2(math.sin(theta - yaw_ref), math.cos(theta - yaw_ref))
                # Compute velocity error.
                vel_error = u_seq[t, 0] - ref_vel

                # --- Compute lateral error in robot frame ---
                dx = x_ref - x
                dy = y_ref - y
                e_y = -math.sin(theta) * dx + math.cos(theta) * dy

                cost += (temp_w_dist * (pos_error ** 2) +
                         self.w_yaw * (heading_error ** 2) +
                         self.w_vel * (vel_error ** 2) +
                         self.w_lat * (e_y ** 2) +
                         self.w_control * (u_seq[t, 0] ** 2 + u_seq[t, 1] ** 2))
            # Penalize rapid changes in steering (steering rate).
            for t in range(1, T):
                delta_rate = u_seq[t, 1] - u_seq[t-1, 1]
                cost += self.w_delta_rate * (delta_rate ** 2)
            return cost

        sol = minimize(objective, u0, bounds=bounds, method="SLSQP", options={'maxiter': 30})
        if(self.last_is_pushing):
            self.max_v = self.max_v_push
            self.is_pushing = True
        else:
            self.max_v = self.max_v_nonpush
            self.is_pushing = False
        if sol.success:

            u_opt = sol.x.reshape((T, 2))
            return u_opt[0]
        else:
            rospy.logwarn("MPC optimization failed: %s", sol.message)
            return self.last_cmd

if __name__ == '__main__':
    try:
        node = MPCControllerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass