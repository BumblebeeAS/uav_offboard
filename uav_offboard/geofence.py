"""Download custom PX4 fences using the node's shared MAVSDK connection."""

import asyncio
from math import nan

from bb_uav_msgs.msg import Geofence, GeofenceCircle, GeofencePolygon
from geographic_msgs.msg import GeoPoint
from mavsdk.asyncio.plugins.geofence import GeofenceAsync
from mavsdk.plugins.geofence import FenceType
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def geo_point(point):
    """Preserve WGS84 coordinates without inventing a fence altitude."""
    return GeoPoint(
        latitude=float(point.latitude_deg),
        longitude=float(point.longitude_deg),
        altitude=nan,
    )


def to_geofence(data, stamp):
    """Preserve all inclusion/exclusion shapes, including an empty fence."""
    return Geofence(
        stamp=stamp,
        polygons=[
            GeofencePolygon(
                inclusion=polygon.fence_type == FenceType.INCLUSION,
                vertices=[geo_point(point) for point in polygon.points],
            )
            for polygon in data.polygons
        ],
        circles=[
            GeofenceCircle(
                inclusion=circle.fence_type == FenceType.INCLUSION,
                center=geo_point(circle.point),
                radius_m=float(circle.radius),
            )
            for circle in data.circles
        ],
    )


class GeofencePublisher:
    """Poll without uploading fences or sending actuator commands."""

    def __init__(self, node, connect):
        self.node = node
        self.connect = connect
        topic = node.declare_parameter('geofence_topic', 'geofence').value
        self.interval = node.declare_parameter('geofence_refresh_interval', 1.0).value
        self.timeout = node.declare_parameter('geofence_download_timeout', 15.0).value
        if self.interval <= 0 or self.timeout <= 0:
            raise ValueError('Geofence refresh interval and timeout must be positive')
        self.publisher = node.create_publisher(
            Geofence, topic,
            QoSProfile(
                depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )

    async def refresh(self):
        """Publish only a completed download; failures leave the old stamp intact."""
        drone = await self.connect()
        plugin = GeofenceAsync(drone)
        try:
            data = await asyncio.wait_for(plugin.download_geofence(), self.timeout)
        finally:
            plugin.destroy()
        self.publisher.publish(to_geofence(data, self.node.get_clock().now().to_msg()))

    async def run(self):
        """Retry discovery and downloads after transient connection failures."""
        while True:
            try:
                await self.refresh()
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                self.node.get_logger().warning(f'Geofence download failed: {exc}')
            await asyncio.sleep(self.interval)
