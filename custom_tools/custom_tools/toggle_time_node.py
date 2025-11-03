#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import subprocess
from typing import Tuple

class ToggleTimeNode(Node):
    def __init__(self):
        super().__init__('toggle_time_node')

        self.declare_parameter('world_name', 'large_demo')
        self.declare_parameter('sun_name', 'sun')

        self.world = self.get_parameter('world_name').get_parameter_value().string_value
        self.sun = self.get_parameter('sun_name').get_parameter_value().string_value

        # Track current mode (False = day, True = night)
        self.is_night = False

        self.req_night = (
            f'name:"{self.sun}" type:DIRECTIONAL '
            'diffuse:{r:0 g:0 b:0 a:1} '
            'specular:{r:0 g:0 b:0 a:1} '
            'intensity:0 cast_shadows:false'
        )

        self.req_day = (
            f'name:"{self.sun}" type:DIRECTIONAL '
            'diffuse:{r:0.8 g:0.8 b:0.8 a:1} '
            'specular:{r:0.8 g:0.8 b:0.8 a:1} '
            'direction:{x:-0.5 y:0.1 z:-0.9} '
            'intensity:5 cast_shadows:false'
        )

        self.srv = self.create_service(Trigger, 'toggle_time', self.handle_toggle)
        self.get_logger().info(
            f"Ready: call /toggle_time to switch Day<->Night "
            f"(world={self.world}, sun={self.sun})"
        )

    def _ign_light_config(self, req_str: str) -> Tuple[bool, str]:
        service = f"/world/{self.world}/light_config"
        cmd = [
            'ign', 'service',
            '-s', service,
            '--reqtype', 'ignition.msgs.Light',
            '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '3000',
            '--req', req_str
        ]
        out = subprocess.run(cmd, capture_output=True, text=True, check=False)
        ok = ("data: true" in out.stdout) or ("data: true" in out.stderr)
        msg = (out.stdout + out.stderr).strip()
        return ok, msg

    def handle_toggle(self, request, response):
        target = "night" if not self.is_night else "day"
        req_str = self.req_night if not self.is_night else self.req_day

        self.get_logger().info(f"Setting {target}…")
        ok, msg = self._ign_light_config(req_str)

        if ok:
            self.is_night = not self.is_night
            response.success = True
            response.message = f"Switched to {target}. ign says: {msg}"
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = f"Failed to set {target}. ign says: {msg}"
            self.get_logger().error(response.message)

        return response

def main():
    rclpy.init()
    node = ToggleTimeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
