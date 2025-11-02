#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_srvs.srv import Empty


class BatterySimNode(Node):

    def __init__(self):
        super().__init__('battery_sim')


        self.declare_parameter('initial_percentage', 100.0)     
        self.declare_parameter('idle_drain_per_hour', 600.0)      # % / hour
        self.declare_parameter('linear_drain_per_meter', 0.4)  # % / meter
        self.declare_parameter('angular_drain_per_rad', 0.5)  # % / rad
        self.declare_parameter('publish_topic', 'battery_state')
        self.declare_parameter('publish_rate_hz', 0.5)
        self.declare_parameter('velocity_topic', '/cmd_vel') 
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('voltage_full', 12.6)           
        self.declare_parameter('voltage_empty', 10.0)           

        # Resolve params
        self.percentage = float(self.get_parameter('initial_percentage').get_parameter_value().double_value)
        self.idle_drain_per_hour = float(self.get_parameter('idle_drain_per_hour').get_parameter_value().double_value)
        self.linear_drain_per_meter = float(self.get_parameter('linear_drain_per_meter').get_parameter_value().double_value)
        self.angular_drain_per_rad = float(self.get_parameter('angular_drain_per_rad').get_parameter_value().double_value)
        self.publish_topic = self.get_parameter('publish_topic').get_parameter_value().string_value
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').get_parameter_value().double_value)
        self.velocity_topic = self.get_parameter('velocity_topic').get_parameter_value().string_value
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.voltage_full = float(self.get_parameter('voltage_full').get_parameter_value().double_value)
        self.voltage_empty = float(self.get_parameter('voltage_empty').get_parameter_value().double_value)

        # Internal state
        self._last_time: Optional[Time] = None
        self._last_twist: Optional[Twist] = None

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.pub = self.create_publisher(BatteryState, self.publish_topic, qos)

    
        self.get_logger().info(f"BatterySim: using Twist from '{self.velocity_topic}' (e.g., Nav2 cmd_vel)")
        self._twist_sub = self.create_subscription(Twist, self.velocity_topic, self._on_twist, qos)

        self.reset_srv = self.create_service(Empty, 'reset_battery', self._on_reset)

        period = 1.0 / max(self.publish_rate_hz, 1e-3)
        self.timer = self.create_timer(period, self._on_timer)

        self.get_logger().info("BatterySim: started at %.1f%%" % self.percentage)


    def _on_twist(self, msg: Twist):
        self._last_twist = msg

    def _on_odom(self, msg: Odometry):
        self._last_twist = msg.twist.twist

    def _on_reset(self, _req, _resp):
        self.percentage = 100.0
        self.get_logger().info("BatterySim: reset to 100%")
        return _resp  # Empty response OK

    def _on_timer(self):
        now = self.get_clock().now().to_msg()

        # Compute dt
        if self._last_time is None:
            self._last_time = now
            # Publish initial state immediately
            self._publish(now)
            return

        dt = self._secs_between(self._last_time, now)
        self._last_time = now
        if dt <= 0.0:
            return

        # Get current speeds (m/s and rad/s). but otherwise assume stationary.
        vx = vy = wz = 0.0
        if self._last_twist is not None:
            vx = float(self._last_twist.linear.x)
            vy = float(self._last_twist.linear.y)
            wz = float(self._last_twist.angular.z)

        idle_pp = self.idle_drain_per_hour * (dt / 3600.0)

        linear_speed = math.hypot(vx, vy)          
        distance = linear_speed * dt               
        rotation = abs(wz) * dt                      

        linear_pp = self.linear_drain_per_meter * distance
        angular_pp = self.angular_drain_per_rad * rotation

        dpp = idle_pp + linear_pp + angular_pp

        # Update state
        prev_pct = self.percentage
        self.percentage = max(0.0, prev_pct - dpp)

        # pnblish only when changed small to avoid spam
        if abs(self.percentage - prev_pct) > 1e-4:
            self._publish(now)


    @staticmethod
    def _secs_between(t0: Time, t1: Time) -> float:
        return (t1.sec - t0.sec) + (t1.nanosec - t0.nanosec) * 1e-9

    def _publish(self, stamp: Time):
        msg = BatteryState()
        msg.header.stamp = stamp

        pct01 = max(0.0, min(1.0, self.percentage / 100.0))
        msg.percentage = pct01

        msg.voltage = self.voltage_empty + (self.voltage_full - self.voltage_empty) * pct01

        msg.current = float('nan')   
        msg.charge = float('nan')
        msg.capacity = float('nan')
        msg.design_capacity = float('nan')
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        msg.present = True
        msg.location = "base_link"
        msg.serial_number = "sim-battery-001"

        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = BatterySimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
