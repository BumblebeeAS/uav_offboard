import asyncio
from unittest.mock import AsyncMock, Mock

import pytest
from builtin_interfaces.msg import Time
from mavsdk.plugins.geofence import GeofenceData

from uav_offboard import geofence as module


@pytest.mark.parametrize('failure', [None, RuntimeError('link lost'), asyncio.TimeoutError()])
def test_download_failure_does_not_publish_empty_or_leak_plugin(monkeypatch, failure):
    plugin = Mock(download_geofence=AsyncMock())
    if failure is None:
        plugin.download_geofence.return_value = GeofenceData()
    else:
        plugin.download_geofence.side_effect = failure
    factory = Mock(return_value=plugin)
    monkeypatch.setattr(module, 'GeofenceAsync', factory)
    node = Mock()
    node.get_clock().now().to_msg.return_value = Time(sec=123)
    node.declare_parameter.side_effect = lambda name, default: Mock(value=default)
    publisher = module.GeofencePublisher(node, AsyncMock())
    if failure is None:
        asyncio.run(publisher.refresh())
        publisher.publisher.publish.assert_called_once()
        message = publisher.publisher.publish.call_args.args[0]
        assert not message.polygons and not message.circles
    else:
        with pytest.raises(type(failure)):
            asyncio.run(publisher.refresh())
        publisher.publisher.publish.assert_not_called()
    plugin.destroy.assert_called_once()
    qos = node.create_publisher.call_args.args[2]
    assert qos.depth == 1
    assert qos.durability == module.DurabilityPolicy.TRANSIENT_LOCAL


def test_cancelled_download_cleans_up_without_publishing(monkeypatch):
    async def download():
        raise asyncio.CancelledError()

    plugin = Mock(download_geofence=download)
    monkeypatch.setattr(module, 'GeofenceAsync', Mock(return_value=plugin))
    node = Mock()
    node.declare_parameter.side_effect = lambda name, default: Mock(value=default)
    publisher = module.GeofencePublisher(node, AsyncMock())
    with pytest.raises(asyncio.CancelledError):
        asyncio.run(publisher.refresh())
    plugin.destroy.assert_called_once()
    publisher.publisher.publish.assert_not_called()
