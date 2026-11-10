"""Search disks fit the uploaded fence; unavailable or changing bounds stop motion."""
import asyncio

import numpy as np
import pytest
from bb_uav_msgs.action import SpiralSearch
from bb_uav_msgs.msg import Geofence, GeofenceCircle, GeofencePolygon
from geographic_msgs.msg import GeoPoint
from pygeodesy.ltp import LocalCartesian
from rclpy.action import GoalResponse

from test_spiral_search import setup_node
from uav_offboard.utils.geofence import bounded_spiral_radius

REFERENCE = (1.2809, 103.8555, 1)


def point(north=0., east=0.):
    p = LocalCartesian(*REFERENCE[:2], 0.).reverse(east, north, 0.)
    return GeoPoint(latitude=p.lat, longitude=p.lon)


def polygon(vertices, inclusion=True):
    return GeofencePolygon(inclusion=inclusion, vertices=[point(*p) for p in vertices])


def circle(radius, north=0., east=0., inclusion=True):
    return GeofenceCircle(center=point(north, east), radius_m=float(radius), inclusion=inclusion)


@pytest.mark.parametrize('fence,centre,expected', [
    (Geofence(circles=[circle(10)]), [8., 0., -5.], 1.5),
    (Geofence(polygons=[polygon([[-5,-5],[5,-5],[5,5],[-5,5]])]), [4.,0.,-5.], .5),
    (Geofence(circles=[circle(20), circle(1, north=3, inclusion=False)]), [0.,0.,-5.], 1.5),
    (Geofence(polygons=[polygon([[2,-1],[4,-1],[4,1],[2,1]], False)]), [0.,0.,-5.], 1.5),
    (Geofence(polygons=[polygon([[0,0],[4,0],[4,1],[1,1],[1,4],[0,4]])]), [.5,.5,-5.], .25),
])
def test_circle_polygon_exclusion_and_concave_clearance(fence, centre, expected):
    margin = .25 if expected == .25 else .5
    radius = bounded_spiral_radius(fence, REFERENCE, centre, 5., margin)
    assert radius == pytest.approx(expected, abs=1e-5)


@pytest.mark.parametrize('fence', [Geofence(), Geofence(circles=[circle(1)]),
                                  Geofence(polygons=[polygon([[0,0],[1,1]])])])
def test_empty_outside_or_invalid_fence_cannot_start(fence):
    with pytest.raises(ValueError):
        bounded_spiral_radius(fence, REFERENCE, [2.,0.,-5.], 5., .5)


@pytest.mark.parametrize('case', ['missing', 'stale', 'future', 'missing_reference', 'valid'])
def test_goal_requires_current_fence_and_global_local_reference(monkeypatch, case):
    node, handle = setup_node(monkeypatch)
    handle.request = SpiralSearch.Goal(radius=5., turn_spacing=1., speed=1.,
                                      settle_threshold=.1, settle_timeout=1.)
    if case == 'missing':node._geofence = None
    elif case == 'stale':node._geofence.stamp.sec = -1001
    elif case == 'future':node._geofence.stamp.sec = 1
    elif case == 'missing_reference':node._local_reference = None
    expected = GoalResponse.ACCEPT if case == 'valid' else GoalResponse.REJECT
    assert node.spiral_search_goal_callback(handle.request) == expected


@pytest.mark.parametrize('change', ['lost', 'tightened', 'reference_reset'])
def test_fence_loss_or_reference_change_aborts_and_holds_current_position(monkeypatch, change):
    node, handle = setup_node(monkeypatch)
    handle.request = SpiralSearch.Goal(radius=.1, turn_spacing=.1, speed=.2,
                                      settle_threshold=.02, settle_timeout=.2)
    node.nav_state, node.arming_state = 14, 2
    held = []
    node.publish_trajectory_setpoint = lambda: held.append(node.absolute_target.copy())
    advance = node.create_rate.return_value.sleep
    def change_bounds():
        advance()
        if change == 'lost':node._geofence = None
        elif change == 'reference_reset':node._local_reference = (*REFERENCE[:2], 2)
        else:
            node._geofence = Geofence(circles=[circle(.51, north=12, east=23)])
    node.create_rate.return_value.sleep = change_bounds
    result = asyncio.run(node.execute_spiral_search_callback(handle)).result
    assert not result.success and node.absolute_target is None
    handle.abort.assert_called_once()
    np.testing.assert_allclose(held[-1], node.current_position)
