import asyncio
from types import SimpleNamespace
from unittest.mock import AsyncMock, Mock

import pytest

from uav_offboard import actuator_control_node as module


def make_node():
    return SimpleNamespace(
        mavsdk=SimpleNamespace(
            add_any_connection=AsyncMock(), first_autopilot=AsyncMock()
        ),
        drone=None, action=None, connection_added=False,
        address="udpin://127.0.0.1:14540", timeout=0.1,
        get_logger=lambda: Mock(),
    )


def test_connection_is_added_once_and_plugin_is_reused(monkeypatch):
    node = make_node()
    node.mavsdk.first_autopilot.return_value = SimpleNamespace(
        is_connected=AsyncMock(return_value=True)
    )
    factory = Mock()
    monkeypatch.setattr(module, "ActionAsync", factory)

    async def run():
        await module.ActuatorControlNode._connect(node)
        await module.ActuatorControlNode._connect(node)

    asyncio.run(run())
    node.mavsdk.add_any_connection.assert_awaited_once_with(node.address)
    node.mavsdk.first_autopilot.assert_awaited_once_with(node.timeout)
    factory.assert_called_once_with(node.drone)


def test_discovery_timeout_does_not_create_action(monkeypatch):
    node = make_node()
    node.mavsdk.first_autopilot.return_value = None
    factory = Mock()
    monkeypatch.setattr(module, "ActionAsync", factory)
    with pytest.raises(TimeoutError, match="No PX4 autopilot"):
        asyncio.run(module.ActuatorControlNode._connect(node))
    factory.assert_not_called()


@pytest.mark.parametrize("enabled,value", [(True, 1), (False, -1)])
def test_actuator_indexes_and_values_are_preserved(enabled, value):
    action = SimpleNamespace(set_actuator=AsyncMock())
    node = SimpleNamespace(action=action, actuation_indexes=[1, 3], ON=1, OFF=-1)
    asyncio.run(module.ActuatorControlNode._actuate(node, enabled))
    assert action.set_actuator.await_count == 2
    action.set_actuator.assert_any_await(1, value)
    action.set_actuator.assert_any_await(3, value)
