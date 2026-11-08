"""RC kill decoding matches PX4; missing configuration/input never releases."""
import asyncio
from dataclasses import replace
import json
import time
from types import SimpleNamespace
from unittest.mock import AsyncMock, Mock

import pytest
from px4_msgs.msg import ManualControlSetpoint, ManualControlSwitches as Switches
from uav_offboard.rc_kill import KillCalibration, RcKillPublisher

CAL = KillCalibration(7, 1000., 1500., 2000., 10., 1., .75)


@pytest.mark.parametrize('raw,state', [(1000,3),(1490,3),(1500,3),(1510,3),(1755,3),(1756,1),(2000,1)])
def test_calibrated_threshold_including_deadzone_and_boundary(raw, state):
    assert CAL.decode({'chancount':8, 'chan7_raw':raw}) == state


@pytest.mark.parametrize('reverse,threshold,low,high', [
    (1.,.75,3,1), (-1.,.75,1,3), (1.,-.75,1,3), (-1.,-.75,3,1)])
def test_channel_reverse_and_threshold_polarity(reverse,threshold,low,high):
    cal = replace(CAL, reverse=reverse, threshold=threshold)
    assert cal.decode({'chancount':8, 'chan7_raw':1000}) == low
    assert cal.decode({'chancount':8, 'chan7_raw':2000}) == high


@pytest.mark.parametrize('fields', [{}, {'chancount':6,'chan7_raw':1000},
                                  {'chancount':8,'chan7_raw':65535},
                                  {'chancount':8,'chan7_raw':0},
                                  {'chancount':8,'chan7_raw':float('nan')}])
def test_missing_or_invalid_channel_is_unknown(fields):
    assert CAL.decode(fields) == Switches.SWITCH_POS_NONE


@pytest.mark.parametrize('values', [{'channel':0},{'channel':19},{'deadzone':500.},
                                  {'threshold':2.},{'minimum':1500.},{'reverse':float('nan')}])
def test_invalid_calibration_is_rejected(values):
    with pytest.raises(ValueError):replace(CAL, **values)


def publisher():
    node = Mock()
    node.declare_parameter.side_effect = lambda name, default: SimpleNamespace(value=default)
    node.get_clock().now().nanoseconds = 123000000
    bridge = RcKillPublisher(node, AsyncMock())
    bridge.calibration = CAL
    bridge.config_received = time.monotonic()
    bridge.parameter_types = {f'RC7_{s}':'float' for s in ('MIN','TRIM','MAX','DZ','REV')}
    bridge.on_manual(ManualControlSetpoint(valid=True,data_source=ManualControlSetpoint.SOURCE_RC))
    return bridge


def rc_message(raw=1000, system=1):
    return SimpleNamespace(message_name='RC_CHANNELS',system_id=system,component_id=1,
                           fields_json=json.dumps({'chancount':8,'chan7_raw':raw}))


@pytest.mark.parametrize('condition', ['valid','rc_lost','stale_rc','stale_config','no_config','wrong_source'])
def test_only_live_valid_rc_and_configuration_can_report_off(condition):
    bridge = publisher()
    if condition=='rc_lost':bridge.on_manual(ManualControlSetpoint(valid=False))
    if condition=='stale_rc':bridge.manual_received -= 3.
    if condition=='stale_config':bridge.config_received -= 16.
    if condition=='no_config':bridge.calibration = None
    if condition=='wrong_source':bridge.on_manual(ManualControlSetpoint(valid=True,data_source=2))
    bridge.on_message(rc_message(), 1)
    state = bridge.publisher.publish.call_args.args[0].kill_switch
    assert state == (Switches.SWITCH_POS_OFF if condition=='valid' else Switches.SWITCH_POS_NONE)


def test_other_vehicles_are_ignored_and_parameter_change_invalidates_old_calibration():
    bridge = publisher()
    bridge.on_message(rc_message(system=2), 1)
    bridge.publisher.publish.assert_not_called()
    change = SimpleNamespace(message_name='PARAM_VALUE',system_id=1,component_id=1,
        fields_json=json.dumps({'param_id':'RC_KILLSWITCH_TH','param_value':-.75}))
    bridge.on_message(change, 1)
    assert bridge.calibration is None
    bridge.on_message(rc_message(), 1)
    assert bridge.publisher.publish.call_args.args[0].kill_switch == Switches.SWITCH_POS_NONE


def test_normal_parameter_read_responses_do_not_clear_calibration():
    bridge = publisher()
    msg = SimpleNamespace(message_name='PARAM_VALUE',system_id=1,component_id=1,
        fields_json=json.dumps({'param_id':'RC_KILLSWITCH_TH','param_value':.75}))
    bridge.on_message(msg,1)
    assert bridge.calibration == CAL


def test_configuration_is_read_from_px4_without_setting_parameters():
    bridge = publisher()
    params = Mock(get_param_int=AsyncMock(return_value=7), get_param_float=AsyncMock(
        side_effect=[1000.,1500.,2000.,10.,1.,.75]))
    cal = asyncio.run(bridge.read_calibration(params))
    assert cal == CAL
    assert [c.args[0] for c in params.get_param_float.call_args_list] == [
        'RC7_MIN','RC7_TRIM','RC7_MAX','RC7_DZ','RC7_REV','RC_KILLSWITCH_TH']
    params.set_param_int.assert_not_called()
    params.set_param_float.assert_not_called()


def test_stream_uses_async_system_id_and_shared_connection_and_cleans_up(monkeypatch):
    from uav_offboard import rc_kill as module
    bridge = publisher()
    drone = SimpleNamespace(_system=object(), get_system_id=AsyncMock(return_value=1),
                            is_connected=AsyncMock(return_value=False))
    bridge.connect = AsyncMock(return_value=drone)
    bridge.refresh_configuration = AsyncMock()
    direct = Mock()
    direct.subscribe_message.side_effect = [11,12]
    monkeypatch.setattr(module, 'MavlinkDirect', Mock(return_value=direct))
    params = Mock()
    monkeypatch.setattr(module, 'ParamAsync', Mock(return_value=params))
    async def run():
        with pytest.raises(asyncio.TimeoutError):
            await asyncio.wait_for(bridge.run(), .1)
    asyncio.run(run())
    drone.get_system_id.assert_awaited_once()
    module.MavlinkDirect.assert_called_once_with(drone._system)
    fields = json.loads(direct.send_message.call_args.args[0].fields_json)
    assert fields['target_system'] == 1
    assert fields['command'] == 511 and fields['param1'] == 65
    assert direct.unsubscribe_message.call_count == 2
    direct.destroy.assert_called_once()
    params.destroy.assert_called_once()


def test_new_px4_without_deadzone_and_integer_reverse_uses_catalog():
    bridge = publisher()
    bridge.parameter_types = None
    floats = [SimpleNamespace(name=n) for n in ('RC7_MIN','RC7_TRIM','RC7_MAX','RC_KILLSWITCH_TH')]
    integers = [SimpleNamespace(name=n) for n in ('RC_MAP_KILL_SW','RC7_REV')]
    params = Mock(get_all_params=AsyncMock(return_value=SimpleNamespace(float_params=floats,int_params=integers)),
                  get_param_int=AsyncMock(side_effect=[7,-1]),
                  get_param_float=AsyncMock(side_effect=[1000.,1500.,2000.,.75]))
    cal = asyncio.run(bridge.read_calibration(params))
    assert cal.deadzone == 0. and cal.reverse == -1
    assert 'RC7_DZ' not in [c.args[0] for c in params.get_param_float.call_args_list]
    assert [c.args[0] for c in params.get_param_int.call_args_list] == ['RC_MAP_KILL_SW','RC7_REV']


@pytest.mark.parametrize('channel', [7, 11])
def test_px4_bytewise_integer_parameter_read_does_not_invalidate_configuration(channel):
    import struct
    bridge = publisher()
    bridge.calibration = replace(CAL, channel=channel)
    raw = struct.unpack('<f', struct.pack('<i', channel))[0]
    msg = SimpleNamespace(message_name='PARAM_VALUE',system_id=1,component_id=1,
        fields_json=json.dumps({'param_id':'RC_MAP_KILL_SW','param_type':6,'param_value':raw}))
    bridge.on_message(msg,1)
    assert bridge.calibration.channel == channel
    assert bridge.publisher.publish.call_count == 0
