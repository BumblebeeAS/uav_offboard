"""Check home-frame conversion without connecting to a vehicle."""

import asyncio
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
from px4_msgs.msg import HomePosition, VehicleLocalPosition

from uav_offboard.offboard_node import OffboardNode


@pytest.fixture
def node():
    instance = OffboardNode.__new__(OffboardNode)
    instance.use_home_position = True
    instance.current_position = np.array([12.0, 23.0, -7.0])
    instance.home_position = None
    instance.position_valid = True
    return instance


def goal(relative=False):
    return SimpleNamespace(x=0.0, y=0.0, z=-2.0, relative=relative)


def test_home_goal_and_feedback(node):
    node.home_position_callback(HomePosition(x=10.0, y=20.0, z=-5.0, valid_lpos=True))
    np.testing.assert_allclose(node.resolve_target(goal()), [10, 20, -7])
    np.testing.assert_allclose(node.feedback_position, [2, 3, -2])


def test_relative_takeoff_does_not_require_home(node):
    np.testing.assert_allclose(node.resolve_target(goal(relative=True)), [12, 23, -9])


def test_missing_or_invalid_home(node):
    for message in (
        HomePosition(valid_lpos=False),
        HomePosition(x=float('nan'), valid_lpos=True),
    ):
        node.home_position_callback(message)
        with pytest.raises(ValueError, match='home'):
            node.resolve_target(goal())
        assert np.isnan(node.feedback_position).all()


def test_home_update_does_not_move_resolved_target(node):
    node.home_position_callback(HomePosition(x=10.0, y=20.0, z=-5.0, valid_lpos=True))
    node.absolute_target = node.resolve_target(goal())
    node.home_position_callback(HomePosition(x=30.0, y=40.0, z=-8.0, valid_lpos=True))
    np.testing.assert_allclose(node.absolute_target, [10, 20, -7])
    np.testing.assert_allclose(node.resolve_target(goal()), [30, 40, -10])


def test_local_position_validity(node):
    for message in (
        VehicleLocalPosition(xy_valid=False, z_valid=True),
        VehicleLocalPosition(xy_valid=True, z_valid=False),
        VehicleLocalPosition(x=float('nan'), xy_valid=True, z_valid=True),
    ):
        node.local_position_callback(message)
        assert not node.position_valid
    node.local_position_callback(VehicleLocalPosition(xy_valid=True, z_valid=True))
    assert node.position_valid


@pytest.mark.parametrize('relative', [False, True])
def test_raw_local_mode_does_not_require_home(node, relative):
    node.use_home_position = False
    target = node.resolve_target(goal(relative=relative))
    expected = [12, 23, -9] if relative else [0, 0, -2]
    np.testing.assert_allclose(target, expected)
    np.testing.assert_allclose(node.feedback_position, node.current_position)


@pytest.mark.parametrize('use_home', [False, True])
@pytest.mark.parametrize('relative', [False, True])
def test_invalid_local_position_rejects_every_goal_frame(node, use_home, relative):
    node.use_home_position = use_home
    node.position_valid = False
    node.home_position_callback(HomePosition(valid_lpos=True))
    node.get_logger = Mock()
    request = goal(relative=relative)
    with pytest.raises(ValueError, match='valid position'):
        node.resolve_target(request)
    assert not node.validate_goal(request)


@pytest.mark.parametrize('invalidate', ['local', 'home'])
def test_reference_lost_after_acceptance_aborts_execution(node, invalidate):
    node.get_logger = Mock()
    node.get_clock = Mock()
    node.is_goal_active = False
    node.home_position_callback(HomePosition(valid_lpos=True))
    request = goal()
    request.x_threshold = request.y_threshold = request.z_threshold = 0.1
    assert node.validate_goal(request)
    if invalidate == 'local':
        node.position_valid = False
    else:
        node.home_position_callback(HomePosition(valid_lpos=False))
    handle = Mock()
    feedback = Mock()
    result = asyncio.run(node.execute_callback(handle, request, feedback))
    assert not result.success
    assert result.message == (
        'Failed to get valid position data' if invalidate == 'local'
        else 'No valid PX4 home position received'
    )
    handle.abort.assert_called_once()
    feedback.assert_not_called()
    assert node.absolute_target is None
    assert not node.is_goal_active
