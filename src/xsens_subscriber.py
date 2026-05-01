#!/usr/bin/env python3
from math import radians, cos, isnan, degrees, hypot
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PointStamped, QuaternionStamped
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3Stamped


class XsensLocalXY(Node):
    """
    Subscribes to xsens lat/lon coordinates (fixes) and quaternion heading.
    Then converts it to (x,y) coordinates in meters.

    *Heading angle in degrees is derived from quaternion

    Notes:
    - First lat/lon is set as the origin (0, 0). Subsequent points are relative to that origin.
    - x = east-west
    - y = north-south
    - heading_deg: north=0, east=90, south=180, west=270
    - Points between consecutive lat/lon coordinates are logged

    Verification notes / assumptions:
    - Conversion uses a simple equirectangular approximation:
        x = R * dlon * cos(lat0), y = R * dlat
      This is accurate for small areas (tens of kilometers at most). For large distances use a geodesic.
    - The very first NavSatFix received (non-NaN) becomes the origin (0,0). If you want a different origin,
      publish a synthetic first fix at the desired origin before real data.
    - Heading is read from the /filter/euler topic (geometry_msgs/Vector3Stamped). msg.vector.z is yaw
      in degrees using counterclockwise-positive (standard math/ROS) convention.
    - Compass heading (North=0, clockwise-positive) is computed as:
        heading_deg = (heading_offset_deg - euler_z) % 360
      Negating euler_z flips the sign convention; heading_offset_deg re-zeros North.
      Field calibration: sensor reads ~30° pointing North, so heading_offset_deg=30 (default).
    - point_spacing_m controls intermediate published points; extremely small values may generate many messages.
    - The node will raise a runtime error if conversion is attempted before an origin is set (defensive check).
    - For runtime validation: subscribe to /gnss (input) and /gnss_local (output). Move the vehicle ~10m east
      and verify that the published x increases by ~10. Repeat for north to validate y sign and magnitude.
    """

    def __init__(self):
        super().__init__('xsens_local_xy_subscriber')

        # Subscribe to XSENS GPS lat/lon which are published on /gnss
        self.gps_topic = self.declare_parameter('gps_topic', '/gnss').value
        # Subscribe to euler vector (we only care about vector.z = yaw in degrees)
        self.euler_topic = self.declare_parameter('euler_topic', '/filter/euler').value
        # Publish converted local coordinates to a different topic to avoid collision
        self.xy_topic = self.declare_parameter('xy_topic', '/gnss_local').value
        self.heading_topic = self.declare_parameter('heading_topic', '/xsens/heading_deg').value
        self.frame_id = self.declare_parameter('frame_id', 'map').value
        self.log_every_fix = self.declare_parameter('log_every_fix', True).value
        self.point_spacing_m = float(self.declare_parameter('point_spacing_m', 0.1).value)
        # Compass heading = (heading_offset_deg - euler_z) % 360.
        # The XSens yaw is counterclockwise-positive; negating it converts to clockwise-positive
        # (compass) convention. The offset re-zeros North: field data shows euler_z=30 when
        # pointing North, so offset=30 maps that to 0°.
        self.heading_offset_deg = float(self.declare_parameter('heading_offset_deg', 30.0).value)

        self.origin_lat: Optional[float] = None
        self.origin_lon: Optional[float] = None
        self.origin_set = False

        self.prev_x_m: Optional[float] = None
        self.prev_y_m: Optional[float] = None
        self.last_heading_deg = 0.0

        self.xy_pub = self.create_publisher(PointStamped, self.xy_topic, 10)
        self.heading_pub = self.create_publisher(Float64, self.heading_topic, 10)
        self.create_subscription(NavSatFix, self.gps_topic, self.gps_callback, 10)
        # subscribe to euler vector; use euler_callback to read yaw (vector.z)
        self.create_subscription(Vector3Stamped, self.euler_topic, self.euler_callback, 10)

        self.get_logger().info(
            f"Subscribing to GPS: {self.gps_topic}  -> publishing local XY: {self.xy_topic}"
        )

    def euler_callback(self, msg: Vector3Stamped):
        # msg.vector.z is yaw in degrees, counterclockwise-positive (standard math convention).
        # Negate to flip to clockwise-positive (compass convention), then add offset to zero North.
        heading_deg = (self.heading_offset_deg - msg.vector.z) % 360.0

        self.last_heading_deg = heading_deg

        heading_msg = Float64()
        heading_msg.data = heading_deg
        self.heading_pub.publish(heading_msg)

    def gps_callback(self, msg: NavSatFix):
        lat = float(msg.latitude)
        lon = float(msg.longitude)

        # Ignore NaN fixes
        if isnan(lat) or isnan(lon):
            # use the non-deprecated logger method
            self.get_logger().warning('Received NaN lat/lon... skipping fix...')
            return

        if not self.origin_set:
            self.origin_lat = lat
            self.origin_lon = lon
            self.origin_set = True
            x_m, y_m = 0.0, 0.0
            self.prev_x_m, self.prev_y_m = x_m, y_m
            self.get_logger().info(
                f"Origin set to lat={self.origin_lat:.8f}, lon={self.origin_lon:.8f}"
            )
            self.publish_xy(msg, x_m, y_m)
            if self.log_every_fix:
                self.get_logger().info(
                    f"fix lat={lat:.8f}, lon={lon:.8f} -> local x={x_m:.2f} m, y={y_m:.2f} m, "
                    f"heading={self.last_heading_deg:.2f}°"
                )
            return

        x_m, y_m = self.latlon_to_local_xy(lat, lon)
        self.publish_points_between(msg, x_m, y_m)

        if self.log_every_fix:
            self.get_logger().info(
                f"fix lat={lat:.8f}, lon={lon:.8f} -> local x={x_m:.2f} m,  y={y_m:.2f} m,  {self.last_heading_deg:.2f}°"
            )

    def latlon_to_local_xy(self, lat: float, lon: float):
        """
        Convert lat/lon to x,y in meters.

        x = east-west
        y = north-south
        """

        # Defensive check: origin must be set before converting.
        if self.origin_lat is None or self.origin_lon is None:
            raise RuntimeError("Origin not set; cannot convert lat/lon to local x/y")

        # Earth radius in meters
        earth_r = 6378137.0

        lat_rad = radians(lat)
        lon_rad = radians(lon)
        origin_lat_rad = radians(self.origin_lat)
        origin_lon_rad = radians(self.origin_lon)

        dlat = lat_rad - origin_lat_rad
        dlon = lon_rad - origin_lon_rad

        x_m = earth_r * dlon * cos(origin_lat_rad)
        y_m = earth_r * dlat
        return x_m, y_m

    def publish_xy(self, gps_msg: NavSatFix, x_m: float, y_m: float):
        point_msg = PointStamped()
        point_msg.header.stamp = gps_msg.header.stamp
        point_msg.header.frame_id = self.frame_id
        point_msg.point.x = x_m
        point_msg.point.y = y_m
        point_msg.point.z = 0.0
        self.xy_pub.publish(point_msg)

    def publish_points_between(self, gps_msg: NavSatFix, target_x_m: float, target_y_m: float):
        if self.prev_x_m is None or self.prev_y_m is None:
            self.publish_xy(gps_msg, target_x_m, target_y_m)
            self.prev_x_m, self.prev_y_m = target_x_m, target_y_m
            return

        dx = target_x_m - self.prev_x_m
        dy = target_y_m - self.prev_y_m
        distance = hypot(dx, dy)

        if distance < 1e-9:
            self.publish_xy(gps_msg, target_x_m, target_y_m)
            return

        step = max(self.point_spacing_m, 1e-3)
        num_steps = int(distance // step)

        for i in range(1, num_steps + 1):
            ratio = min((i * step) / distance, 1.0)
            xi = self.prev_x_m + ratio * dx
            yi = self.prev_y_m + ratio * dy
            self.publish_xy(gps_msg, xi, yi)
            if self.log_every_fix:
                self.get_logger().info(
                    f"                                                x={xi:.2f} m,  y={yi:.2f} m, "
                    f" {self.last_heading_deg:.2f}°"
                )

        if num_steps == 0 or abs(target_x_m - xi) > 1e-6 or abs(target_y_m - yi) > 1e-6:
            self.publish_xy(gps_msg, target_x_m, target_y_m)

        self.prev_x_m, self.prev_y_m = target_x_m, target_y_m


def main(args=None):
    rclpy.init(args=args)
    node = XsensLocalXY()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()