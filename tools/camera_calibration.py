#!/usr/bin/env python3
"""
camera_calibration.py - measures the ACTUAL view axis of the camera
============================================================= Why needed: <pose>0 0 0.15 0 in
models/swarm_drone_*/model.sdf Which direction the sign of the <pitch> 0</pose> value corresponds to
in Gazebo was not clear through indirect tests. This tool bypasses indirect reasoning completely:

  - Gets the ACTUAL location/orientation of vehicles from Gazebo (/gazebo/model_states), NOT from
  MAVLink. Stream MAVLink can be stale (measured: 9 messages accumulate in 6 s), while pose Gazebo
  is the simulation itself. - Detects purple target from camera simultaneously. - For each
  determination (true rise, true side angle) records the pair <-> (bbox y, bbox x) and fits it
  correctly.

Outputs: - ELEMENT corresponding to the framing center (y=360) = angle of the camera's optical axis
relative to the horizon (with body pitch removed) - scale in degrees/pixel -> expected 12.95/720 =
0.01799 (vertical) 22.81/1280 = 0.01782 (horizontal) If the measured scale is different than
expected, it means the FOV/image setting is inconsistent.
"""

import argparse
import math

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import Image


def q_to_euler(q):
    sr = 2 * (q.w * q.x + q.y * q.z)
    cr = 1 - 2 * (q.x * q.x + q.y * q.y)
    sp = 2 * (q.w * q.y - q.z * q.x)
    sy = 2 * (q.w * q.z + q.x * q.y)
    cy = 1 - 2 * (q.y * q.y + q.z * q.z)
    return (math.atan2(sr, cr), math.asin(max(-1.0, min(1.0, sp))),
            math.atan2(sy, cy))


class Calibration:
    def __init__(self, pursuer_value, target_value, topic, duration_value):
        self.pursuer_label, self.target_label, self.duration_value = pursuer_value, target_value, duration_value
        self.bridge = CvBridge()
        self.state_value = None
        self.samples_item = []
        # ATTENTION: frame counter is required. NO measurement with rospy.wait_for_message() -
        # gazebo_ros_camera casts LAZY, returning OLD cache frame first on each new subscription (measured: 6
        # consecutive wait_for_message returned the same seq=4604). Permanent subscription required.
        self.frame_count_value = 0
        self.last_seq_value = -1
        self.nearest_distance = None
        rospy.init_node('camera_calibration', anonymous=True)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self._state_cb,
                         queue_size=1)
        rospy.Subscriber(topic, Image, self._frame_cb, queue_size=1)
        print(f"resting: {topic} (fighter={pursuer_value} target={target_value})", flush=True)

    def _state_cb(self, msg):
        try:
            ai = msg.name.index(self.pursuer_label)
            hi = msg.name.index(self.target_label)
        except ValueError:
            return
        self.state_value = (msg.pose[ai], msg.pose[hi])

    def _frame_cb(self, msg):
        self.frame_count_value += 1
        self.last_seq_value = msg.header.seq
        if self.state_value is None:
            return
        pursuer_value, target_value = self.state_value
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([140, 120, 60]),
                           np.array([160, 255, 255]))
        if int(mask.sum() // 255) < 15:
            return
        ys, xs = np.where(mask > 0)
        bx, by = float(xs.mean()), float(ys.mean())

        # TRUE orientation of the target relative to the fighter (in world frame Gazebo)
        dx = target_value.position.x - pursuer_value.position.x
        dy = target_value.position.y - pursuer_value.position.y
        dz = target_value.position.z - pursuer_value.position.z
        roll, pitch, yaw = q_to_euler(pursuer_value.orientation)

        # First turn it to the body frame (yaw), then remove pitch/roll: we want to measure the fixed angle of
        # the camera relative to the BODY.
        forward = dx * math.cos(yaw) + dy * math.sin(yaw)
        right_value = -dx * math.sin(yaw) + dy * math.cos(yaw)
        # body pitch/roll de-rotation (not small angle, full transformation)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cr, sr = math.cos(roll), math.sin(roll)
        x_b = forward * cp + dz * sp
        y_b = right_value * cr - (-forward * sp + dz * cp) * sr
        z_b = right_value * sr + (-forward * sp + dz * cp) * cr
        range_horizontal = math.hypot(x_b, y_b)
        if range_horizontal < 5:
            return
        elevation_value2 = math.degrees(math.atan2(z_b, range_horizontal))
        lateral = math.degrees(math.atan2(y_b, x_b))
        self.nearest_distance = (elevation_value2, lateral)
        self.samples_item.append((elevation_value2, lateral, by, bx,
                              math.sqrt(dx * dx + dy * dy + dz * dz)))
        if len(self.samples_item) % 20 == 1:
            print(f"  n= {len(self.samples_item):4d} rise= {elevation_value2:+6.2f} "
                  f"side= {lateral:+6.2f} bbox y= {by:5.0f} x= {bx:5.0f}  "
                  f"range={self.samples_item[-1][4]:6.1f} m", flush=True)

    def report_value(self):
        n = len(self.samples_item)
        print()
        if n < 10:
            print(f"INSUFFICIENT SAMPLE ({n}). Did the target never enter the frame?")
            return
        elevation_value = np.array([o[0] for o in self.samples_item])
        lateral = np.array([o[1] for o in self.samples_item])
        by = np.array([o[2] for o in self.samples_item])
        bx = np.array([o[3] for o in self.samples_item])

        print(f"=== CAMERA CALIBRATION ({n} example) ===")
        # VERTICAL: by = a*rise + b -> y=360 gives rise = axis
        a, b = np.polyfit(elevation_value, by, 1)
        axis_value = (360.0 - b) / a
        print(f"  vertical : bbox_y = {a:+.2f}*rise + {b:.1f}")
        print(f"           scale {abs(1/a):.5f} degrees/pixel "
              f"(expected 12.950 / 720 = 0.01799 )")
        print(f"           >>> IMAGE CENTER = {axis_value:+.2f} degrees elevation "
              f"(according to body)")
        print(f"           SDF'de yazan: models/swarm_drone_1/model.sdf "
              f"pose pitch")
        # HORIZONTAL: bx = c*side + d
        c, d = np.polyfit(lateral, bx, 1)
        center_lateral = (640.0 - d) / c
        print(f"  horizontal : bbox_x = {c:+.2f}*side + {d:.1f}")
        print(f"           scale {abs(1/c):.5f} degrees/pixel "
              f"(expected 22.813 / 1280 = 0.01782 )")
        print(f"           >>> IMAGE CENTER = {center_lateral:+.2f} derece yan")
        print(f"  rise range: {elevation_value.min():+.1f}..{elevation_value.max():+.1f} degrees")
        print(f"  range range: {min(o[4] for o in self.samples_item):.0f}.. "
              f"{max(o[4] for o in self.samples_item):.0f} m")

    scan_value2 = None
    target_alt_value = 0.0

    def run_value(self, mav=None, hold_value=None):
        """If hold = (lat, lon, alt) is given, it will MAINTAIN position and turn the nose to the target.

        Position/yaw commands do not last if left to a separate process (two scans were wasted
        because the yaw follower expired during the measurement); That's why it's done in the same
        cycle.
        """
        from pymavlink import mavutil as mu
        self.t0 = rospy.Time.now()
        last_value = rospy.Time.now()
        deadline = rospy.Time.now() + rospy.Duration(self.duration_value)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if mav is not None and self.state_value is not None:
                pursuer_value, target_value = self.state_value
                if hold_value is not None:
                    alt = hold_value[2]
                    if self.scan_value2:
                        # Slowly scan altitude: no matter where the camera's axis is, the target's elevation CUTS it at one
                        # point.
                        elapsed_item = (rospy.Time.now() - self.t0).to_select()
                        lo, hi, per = self.scan_value2
                        phase_value = (elapsed_item % per) / per
                        alt = lo + (hi - lo) * (1 - abs(2 * phase_value - 1))
                        self.target_alt_value = alt
                    mav.mav.set_position_target_global_int_send(
                        0, mav.target_system, mav.target_component,
                        mu.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                        0b0000111111111000,
                        int(hold_value[0] * 1e7), int(hold_value[1] * 1e7), alt,
                        0, 0, 0, 0, 0, 0, 0, 0)
                direction = math.degrees(math.atan2(
                    target_value.position.y - pursuer_value.position.y,
                    target_value.position.x - pursuer_value.position.x)) % 360
                mav.mav.command_long_send(
                    mav.target_system, mav.target_component,
                    mu.mavlink.MAV_CMD_CONDITION_YAW, 0, direction, 120, 0, 0, 0, 0, 0)
                if (rospy.Time.now() - last_value).to_select() > 20:
                    last_value = rospy.Time.now()
                    dz = target_value.position.z - pursuer_value.position.z
                    horizontal = math.hypot(target_value.position.x - pursuer_value.position.x,
                                       target_value.position.y - pursuer_value.position.y)
                    print(f"  [tracking] range={math.hypot(horizontal,dz):6.0f} m "
                          f"rise={math.degrees(math.atan2(dz,horizontal)):+6.1f} deg "
                          f"command_alt={self.target_alt_value:5.1f} detect={len(self.samples_item)} frame={self.frame_count_value} "
                          f"seq={self.last_seq_value}", flush=True)
            rospy.sleep(0.4)
        self.report_value()


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--pursuer-value', default='iris-1')
    p.add_argument('--target-value', default='target_value')
    p.add_argument('--topic', default='/drone_1/webcam/image_raw')
    p.add_argument('--duration-value', type=float, default=240)
    p.add_argument('--port', type=int, default=0,
                   help='If given, it keeps the hunter in position from this port + turns the nose to the target')
    p.add_argument('--north-value', type=float, default=None, help='North (m) relative to home\'')
    p.add_argument('--east-value', type=float, default=None, help='East (m) from home\'')
    p.add_argument('--alt', type=float, default=None, help='altitude (m)')
    p.add_argument('--scan-value2', default='', metavar='LO,HI,PERIOD',
                   help='change altitude between LO..HI in PERIOD seconds')
    a = p.parse_args()
    k = Calibration(a.pursuer_value, a.target_value, a.topic, a.duration_value)
    if a.scan_value2:
        lo, hi, per = (float(x) for x in a.scan_value2.split(','))
        k.scan_value2 = (lo, hi, per)
        print(f'altitude scan: {lo:.0f} - {hi:.0f} m, {per:.0f} s period')
    mav = hold_value = None
    if a.port:
        from pymavlink import mavutil
        mav = mavutil.mavlink_connection(f'udpin:127.0.0.1:{a.port}', source_system=254)
        mav.wait_heartbeat()
        mav.mav.param_set_send(mav.target_system, mav.target_component,
                               b'WP_YAW_BEHAVIOR', 0,
                               mavutil.mavlink.MAV_PARAM_TYPE_INT8)
        print(f'fighter control: port {a.port} , WP_YAW_BEHAVIOR = 0')
        if None not in (a.north_value, a.east_value, a.alt):
            EARTH_R = 6378137.0
            # Public ArduPilot SITL/CMMAT reference; not the actual field coordinate.
            LAT0, LON0 = -35.363261, 149.165230
            lat = LAT0 + math.degrees(a.north_value / EARTH_R)
            lon = LON0 + math.degrees(a.east_value / (EARTH_R * math.cos(math.radians(LAT0))))
            hold_value = (lat, lon, a.alt)
            print(f'location held: {a.north_value:.0f} m north, {a.east_value:.0f} m east, {a.alt:.0f} m')
    k.run_value(mav, hold_value)


if __name__ == '__main__':
    main()
