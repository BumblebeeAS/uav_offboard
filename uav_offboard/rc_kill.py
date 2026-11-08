"""Decode the RC kill switch from the node's existing MAVLink connection."""

import asyncio
from dataclasses import dataclass
import json
import math
import struct
import time

from mavsdk.asyncio.plugins.param import ParamAsync
from mavsdk.plugins.mavlink_direct import MavlinkDirect, MavlinkMessage
from px4_msgs.msg import ManualControlSetpoint, ManualControlSwitches
from rclpy.qos import qos_profile_sensor_data


@dataclass(frozen=True)
class KillCalibration:
    channel: int
    minimum: float
    trim: float
    maximum: float
    deadzone: float
    reverse: float
    threshold: float

    def __post_init__(self):
        values = (self.minimum, self.trim, self.maximum, self.deadzone,
                  self.reverse, self.threshold)
        if (not 1 <= self.channel <= 18 or not all(math.isfinite(v) for v in values)
                or self.deadzone < 0 or not self.minimum < self.trim-self.deadzone
                or not self.trim+self.deadzone < self.maximum
                or not -1 <= self.threshold <= 1 or self.reverse == 0):
            raise ValueError('Invalid PX4 RC kill-switch calibration')

    def decode(self, fields):
        """Match PX4 rc_update's piecewise calibration and signed threshold."""
        if int(fields.get('chancount', 0)) < max(4, self.channel):
            return ManualControlSwitches.SWITCH_POS_NONE
        raw = float(fields.get(f'chan{self.channel}_raw', float('nan')))
        if not math.isfinite(raw) or not 0 < raw < 65535:
            return ManualControlSwitches.SWITCH_POS_NONE
        if raw < self.trim-self.deadzone:
            scaled = (raw-self.minimum)/(self.trim-self.deadzone-self.minimum)-1.
        elif raw > self.trim+self.deadzone:
            scaled = (raw-self.trim-self.deadzone)/(self.maximum-self.trim-self.deadzone)
        else:
            scaled = 0.
        scaled = max(-1., min(1., scaled))
        if self.reverse < 0:
            scaled = -scaled
        value = .5*scaled+.5
        if self.threshold < 0:
            value = -value
        return (ManualControlSwitches.SWITCH_POS_ON if value > self.threshold
                else ManualControlSwitches.SWITCH_POS_OFF)


class RcKillPublisher:
    """
    Publish fresh switch samples; never arm, unkill, or change parameters.

    RC_CHANNELS carries receiver values even on loss. Require the independent
    PX4 manual-input validity signal before reporting OFF. Configuration is
    periodically re-read; PARAM_VALUE changes immediately invalidate the old
    calibration until a complete read succeeds.
    """

    def __init__(self, node, connect):
        self.node, self.connect = node, connect
        self.topic = node.declare_parameter(
            'rc_switches_topic', '/uav/rc/manual_control_switches').value
        self.publisher = node.create_publisher(
            ManualControlSwitches, self.topic, qos_profile_sensor_data)
        self.subscription = node.create_subscription(
            ManualControlSetpoint, '/fmu/out/manual_control_setpoint', self.on_manual,
            qos_profile_sensor_data)
        self.manual_valid = False
        self.manual_received = None
        self.calibration = None
        self.config_received = None
        self.parameter_types = None
        self.last_state = None

    def on_manual(self, message):
        self.manual_valid = (message.valid
                             and message.data_source == ManualControlSetpoint.SOURCE_RC)
        self.manual_received = time.monotonic()
        if not self.manual_valid:
            self.publish(ManualControlSwitches.SWITCH_POS_NONE)

    def publish(self, state):
        stamp = self.node.get_clock().now().nanoseconds // 1000
        self.publisher.publish(ManualControlSwitches(
            timestamp=stamp, timestamp_sample=stamp, kill_switch=state))
        if state != self.last_state:
            label = {0: 'UNKNOWN', 1: 'ON (killed)', 3: 'OFF (released)'}.get(state, str(state))
            self.node.get_logger().info(f'RC kill switch: {label} -> {self.topic}')
            self.last_state = state

    def on_message(self, message, target_system):
        if message.system_id != target_system or message.component_id != 1:
            return
        fields = json.loads(message.fields_json)
        if message.message_name == 'PARAM_VALUE':
            name = str(fields.get('param_id', '')).rstrip('\x00')
            cal = self.calibration
            if cal is not None:
                expected = {'RC_MAP_KILL_SW': cal.channel, 'RC_KILLSWITCH_TH': cal.threshold}
                expected.update({f'RC{cal.channel}_{suffix}': value for suffix, value in zip(
                    ('MIN', 'TRIM', 'MAX', 'DZ', 'REV'),
                    (cal.minimum, cal.trim, cal.maximum, cal.deadzone, cal.reverse))})
                if name in expected:
                    raw = fields.get('param_value')
                    value = float(raw) if raw is not None else float('nan')
                    # PX4 uses MAVLink byte-wise encoding for INT32 parameters;
                    # the wire float contains integer bits, not a numeric cast.
                    if raw is not None and fields.get('param_type') == 6:
                        value = struct.unpack('<i', struct.pack('<f', value))[0]
                    if value != expected[name]:
                        self.calibration = None
                        self.publish(ManualControlSwitches.SWITCH_POS_NONE)
            return
        if message.message_name != 'RC_CHANNELS':
            return
        now = time.monotonic()
        valid = (self.calibration is not None and self.config_received is not None
                 and now-self.config_received <= 15.
                 and self.manual_valid and self.manual_received is not None
                 and now-self.manual_received <= 2.)
        state = self.calibration.decode(fields) if valid else ManualControlSwitches.SWITCH_POS_NONE
        self.publish(state)

    async def read_calibration(self, params):
        if self.parameter_types is None:
            catalog = await asyncio.wait_for(params.get_all_params(), 30.)
            self.parameter_types = {p.name: 'float' for p in catalog.float_params}
            self.parameter_types.update({p.name: 'int' for p in catalog.int_params})
        channel = await asyncio.wait_for(params.get_param_int('RC_MAP_KILL_SW'), 5.)
        if not 1 <= channel <= 18:
            raise ValueError(f'RC_MAP_KILL_SW={channel}: no supported kill channel mapped')
        # Serialize parameter requests on this shared MAVLink connection.
        values = []
        for suffix in ('MIN', 'TRIM', 'MAX', 'DZ', 'REV'):
            name = f'RC{channel}_{suffix}'
            # New PX4 firmware removed RC*_DZ and changed RC*_REV to int.
            # Only a completed catalog can establish that DZ is absent.
            if suffix == 'DZ' and name not in self.parameter_types:
                values.append(0.)
                continue
            kind = self.parameter_types.get(name)
            if kind is None:
                raise ValueError(f'Missing required RC parameter {name}')
            getter = params.get_param_int if kind == 'int' else params.get_param_float
            values.append(await asyncio.wait_for(getter(name), 5.))
        threshold = await asyncio.wait_for(params.get_param_float('RC_KILLSWITCH_TH'), 5.)
        return KillCalibration(channel, *values, threshold)

    async def refresh_configuration(self, params):
        while True:
            try:
                calibration = await self.read_calibration(params)
                self.calibration = calibration
                self.config_received = time.monotonic()
                self.node.get_logger().info(
                    f'RC kill mapping: channel {calibration.channel}, '
                    f'threshold {calibration.threshold:g}, '
                    f'calibration [{calibration.minimum:g}, {calibration.trim:g}, '
                    f'{calibration.maximum:g}], '
                    f'deadzone {calibration.deadzone:g}, reverse {calibration.reverse:g}',
                    throttle_duration_sec=30.)
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                self.calibration = None
                self.publish(ManualControlSwitches.SWITCH_POS_NONE)
                self.node.get_logger().warning(f'RC kill calibration unavailable: {exc}')
            await asyncio.sleep(5.)

    async def run(self):
        while True:
            direct = params = None
            refresh = None
            handles = []
            try:
                drone = await self.connect()
                target = await drone.get_system_id()
                direct = MavlinkDirect(drone._system)
                params = ParamAsync(drone)
                self.parameter_types = None
                loop = asyncio.get_running_loop()

                def receive(message, _):
                    loop.call_soon_threadsafe(self.handle_message, message, target)
                for name in ('RC_CHANNELS', 'PARAM_VALUE'):
                    handles.append(direct.subscribe_message(name, receive))
                refresh = asyncio.create_task(self.refresh_configuration(params))
                # Request telemetry only. No arming, mode or RC override commands.
                request = MavlinkMessage(
                    message_name='COMMAND_LONG', system_id=245, component_id=191,
                    target_system_id=target, target_component_id=1,
                    fields_json=json.dumps({
                        'target_system': target, 'target_component': 1,
                        'command': 511, 'confirmation': 0, 'param1': 65, 'param2': 100000.,
                        'param3': 0., 'param4': 0., 'param5': 0., 'param6': 0., 'param7': 0.}))
                await asyncio.to_thread(direct.send_message, request)
                while await drone.is_connected():
                    await asyncio.sleep(1.)
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                self.node.get_logger().warning(f'RC kill telemetry unavailable: {exc}')
            finally:
                self.calibration = None
                if refresh is not None:
                    refresh.cancel()
                    await asyncio.gather(refresh, return_exceptions=True)
                if direct is not None:
                    for handle in handles:
                        direct.unsubscribe_message(handle)
                    direct.destroy()
                if params is not None:
                    params.destroy()
            await asyncio.sleep(2.)

    def handle_message(self, message, target):
        try:
            self.on_message(message, target)
        except (ValueError, TypeError, KeyError) as exc:
            self.publish(ManualControlSwitches.SWITCH_POS_NONE)
            self.node.get_logger().warning(
                f'Invalid RC kill telemetry: {exc}', throttle_duration_sec=5.)
