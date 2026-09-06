"""Static contract tests for the GUI ROS frontend."""

from pathlib import Path
import os
from types import SimpleNamespace

import pytest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')


ROOT = Path(__file__).parents[1] / 'robot_manual_gui'


def test_gui_never_imports_dynamixel_sdk():
    source = ''.join(path.read_text(encoding='utf-8')
                     for path in ROOT.glob('*.py'))
    assert 'dynamixel_sdk' not in source
    assert 'write1Byte' not in source
    assert 'write2Byte' not in source
    assert 'write4Byte' not in source


def test_gui_uses_existing_control_interfaces():
    source = (ROOT / 'ros_interface.py').read_text(encoding='utf-8')
    for interface in (
            '/arm_controller/joint_trajectory',
            '/gripper_controller/follow_joint_trajectory',
            '/cleaning/enable', '/tool/emergency_stop', '/tool/detached'):
        assert interface in source


def test_mode_status_does_not_overwrite_pending_operator_request():
    source = (ROOT / 'main_window.py').read_text(encoding='utf-8')
    assert 'self.mode_combo.setCurrentText(mode)' not in source


def test_end_effector_scope_blocks_arm_publish_path():
    ros_source = (ROOT / 'ros_interface.py').read_text(encoding='utf-8')
    window_source = (ROOT / 'main_window.py').read_text(encoding='utf-8')
    assert "self.control_scope == 'END_EFFECTOR_ONLY'" in ros_source
    assert 'manual and not end_effector_only' in window_source
    assert 'CONTROL / TEST SCOPE:' in window_source


def test_spur_gui_uses_only_id5_and_requires_explicit_enable():
    ros_source = (ROOT / 'ros_interface.py').read_text(encoding='utf-8')
    window_source = (ROOT / 'main_window.py').read_text(encoding='utf-8')
    assert "'/tool/fsm_command'" in ros_source
    assert "'/tool/calibration_command'" in ros_source
    assert 'command_spur_fsm' in ros_source
    assert 'command_calibration' in ros_source
    spur = ros_source[ros_source.index('    def set_spur_motor_enabled'):ros_source.index(
        '    def command_spur_fsm')]
    assert 'torque_pub' not in spur
    assert "ENABLE ID5" in window_source
    assert "DISABLE ID5" in window_source
    assert "CalibrationSession ENABLE ID5 requested" in window_source


def test_spur_calibration_uses_actual_register_state_and_no_fake_endpoints():
    bridge = (Path(__file__).parents[2] / 'dynamixel_control' /
              'dynamixel_control' / 'moveit_dynamixel_bridge.py').read_text(
                  encoding='utf-8')
    window = (ROOT / 'main_window.py').read_text(encoding='utf-8')
    assert "ADDR_TORQUE_ENABLE" in bridge
    assert "'tool_torque_state'" in bridge
    assert "'/tool/calibration_command'" in bridge
    assert 'CalibrationSession' in bridge
    assert 'spur gripper action rejected' in bridge
    assert "MOTOR −1°" in window
    assert "SET CURRENT AS OPEN" in window
    assert "SET CURRENT AS CLOSE" in window
    jog = window[window.index('def _jog_spur_motor'):window.index('def _capture_spur_endpoint')]
    assert 'temporary_jog_safe_min' not in jog


def test_calibration_mode_has_no_startup_configuration_writes():
    bridge = (Path(__file__).parents[2] / 'dynamixel_control' /
              'dynamixel_control' / 'moveit_dynamixel_bridge.py').read_text(
                  encoding='utf-8')
    start = bridge.index('elif self.calibration_jog_enabled:')
    end = bridge.index('elif self.tool_motion_allowed', start)
    block = bridge[start:end]
    assert 'group_sync_read.addParam(5)' in block
    assert 'write1ByteTxRx' not in block
    assert 'write4ByteTxRx' not in block


def test_spur_bridge_starts_torque_off_and_has_one_gripper_executor():
    bridge = (Path(__file__).parents[2] / 'dynamixel_control' /
              'dynamixel_control' / 'moveit_dynamixel_bridge.py').read_text(
                  encoding='utf-8')
    assert bridge.count('    def execute_gripper(self, goal_handle):') == 1
    assert "self.tool_ids == [5]" in bridge
    assert "self.tool_type == 'dual_motor_gripper'" in bridge


def _window(scope):
    from PyQt5.QtWidgets import QApplication
    from robot_manual_gui.main_window import ManualMainWindow
    from robot_manual_gui.ros_interface import GuiSignals

    app = QApplication.instance() or QApplication([])
    goals = []
    node = SimpleNamespace(
        control_scope=scope, selected_tool='dual_motor_gripper',
        positions={}, efforts={}, gripper_busy=False,
        request_mode=lambda _mode: None, jog_arm=lambda *_args: None,
        command_arm=lambda *_args: None,
        command_gripper=lambda position: (goals.append(position) or True),
        command_tool_fsm=lambda command: (goals.append(command) or True),
        stop_gripper=lambda: None, command_cleaner=lambda *_args: None,
        emergency_stop=lambda: None, tool_detached=lambda: None,
        set_dual_motor_enabled=lambda *_args: True,
        manual_dual_recovery_jog=lambda *_args: True,
        command_dual_calibration=lambda *_args, **_kwargs: True)
    profile = {
        'calibrated': True, 'actuator_ids': [3, 4],
        'open_position': 1.0, 'close_position': 0.0,
        'safe_min_tick': -526, 'safe_max_tick': 2384,
        'motor_endpoints': {
            3: {'open': 1056, 'close': -526},
            4: {'open': 2384, 'close': 839}}}
    return app, ManualMainWindow(node, GuiSignals(), profile, False), goals


def _ready_status(scope):
    return {
        'control_scope': scope, 'tool_type': 'dual_motor_gripper',
        'profile_valid': True, 'calibrated': True,
        'actuators_discovered': True, 'motion_allowed': True,
        'read_only': False, 'emergency_stop': False, 'tool_detached': False,
        'bridge_connected': True,
        'fsm_state': 'READY',
        'endpoint_calibration_verified': True,
        'synchronization': {
            'state': 'SYNCHRONIZED', 'spread': 0.0, 'limit': 0.05},
        'dual_calibration': {'state': 'RECALIBRATION_REQUIRED', 'active': False},
        'actuators': [
            {'id': 3, 'online': True, 'position': 265, 'effort': 10},
            {'id': 4, 'online': True, 'position': 1612, 'effort': 10}]}


def test_dual_open_close_use_fsm_ingress_only_when_all_gates_are_ready():
    _app, window, commands = _window('END_EFFECTOR_ONLY')
    status = _ready_status('END_EFFECTOR_ONLY')
    status['dual_calibration'] = {'state': 'READY', 'active': False}
    for sample in status['actuators']:
        sample.update(torque_state='ON', hardware_error=0)
    window._update_tool_status(status)
    window._update_mode('MANUAL')
    assert window.open_button.isEnabled()
    assert window.close_button.isEnabled()
    window._command_tool('OPEN')
    window._command_tool('CLOSE')
    assert commands == ['OPEN', 'CLOSE']
    window.close()


def test_dual_hold_buttons_use_endpoint_jog_without_calibration():
    _app, window, commands = _window('END_EFFECTOR_ONLY')
    status = _ready_status('END_EFFECTOR_ONLY')
    status['dual_calibration'] = {'state': 'READY', 'active': False}
    for sample in status['actuators']:
        sample.update(torque_state='ON', hardware_error=0)
    window._update_tool_status(status)
    window._update_mode('MANUAL')
    calibration_calls = []
    window.node.command_dual_calibration = (
        lambda command, **values:
        (calibration_calls.append((command, values)) or True))
    window._start_dual_hold_jog('OPEN')
    assert window.dual_hold_jog_active
    assert commands == ['JOG_OPEN']
    assert calibration_calls == []
    window._release_dual_hold_jog()
    assert calibration_calls == []
    assert commands == ['JOG_OPEN', 'HOLD']
    assert not window.dual_hold_jog_active
    window.close()


def test_dual_arrow_hold_uses_endpoint_jog_and_release_hold():
    _app, window, _commands = _window('END_EFFECTOR_ONLY')
    calls = []
    window.node.command_dual_calibration = (
        lambda command, **values: (calls.append((command, values)) or True))
    status = _ready_status('END_EFFECTOR_ONLY')
    status['dual_calibration'] = {'state': 'READY', 'active': False}
    for sample in status['actuators']:
        sample.update(torque_state='ON', hardware_error=0)
    window._update_tool_status(status)
    window._update_mode('MANUAL')
    window._start_dual_key_jog(1)
    assert _commands == ['JOG_OPEN']
    assert calls == []
    window._stop_dual_key_jog()
    assert _commands[-1] == 'HOLD'
    assert not window.dual_key_jog_timer.isActive()
    window.close()


@pytest.mark.parametrize('mutation', ('offline', 'hardware_error', 'torque_off',
                                      'calibration_invalid', 'sync_fault'))
def test_dual_motion_buttons_fail_closed(mutation):
    _app, window, _commands = _window('END_EFFECTOR_ONLY')
    status = _ready_status('END_EFFECTOR_ONLY')
    status['dual_calibration'] = {'state': 'READY', 'active': False}
    for sample in status['actuators']:
        sample.update(torque_state='ON', hardware_error=0)
    if mutation == 'offline':
        status['actuators'][1]['online'] = False
    elif mutation == 'hardware_error':
        status['actuators'][0]['hardware_error'] = 4
    elif mutation == 'torque_off':
        status['actuators'][1]['torque_state'] = 'OFF'
    elif mutation == 'calibration_invalid':
        status['endpoint_calibration_verified'] = False
    elif mutation == 'sync_fault':
        status['synchronization'] = {
            'state': 'FAULT', 'spread': 0.1, 'limit': 0.05}
    window._update_tool_status(status)
    window._update_mode('MANUAL')
    assert not window.open_button.isEnabled()
    assert not window.close_button.isEnabled()
    window.close()


def test_end_effector_scope_enables_only_tool_controls():
    _app, window, _goals = _window('END_EFFECTOR_ONLY')
    window._update_tool_status(_ready_status('END_EFFECTOR_ONLY'))
    window._update_mode('MANUAL')
    assert not window.open_button.isEnabled()
    assert not window.close_button.isEnabled()
    assert window.dual_start_calibration.isEnabled()
    assert window.tool_stop.isEnabled()
    assert not any(widget.isEnabled() for widget in window.arm_buttons)
    window.close()


def test_full_robot_preserves_arm_feedback_gate():
    _app, window, _goals = _window('FULL_ROBOT')
    window._update_tool_status(_ready_status('FULL_ROBOT'))
    window._update_mode('MANUAL')
    assert not any(widget.isEnabled() for widget in window.arm_buttons)
    window.seen_arm_joints.add('arm_joint_1')
    window._refresh_buttons()
    assert all(widget.isEnabled() for widget in window.arm_widgets['arm_joint_1'])
    assert not any(widget.isEnabled()
                   for widget in window.arm_widgets['arm_joint_2'])
    window.close()


def test_jog_interpolates_both_motors_and_busy_blocks_queue():
    _app, window, goals = _window('END_EFFECTOR_ONLY')
    window._update_tool_status(_ready_status('END_EFFECTOR_ONLY'))
    window._update_mode('MANUAL')
    window._jog_gripper(1)
    assert goals == []
    window.gripper_busy = True
    window._jog_gripper(1)
    assert goals == []
    window.close()


@pytest.mark.parametrize('finish', ('release', 'close', 'deactivate', 'stop'))
def test_hold_close_button_stops_timer_and_holds(finish):
    from PyQt5.QtCore import QEvent
    _app, window, commands = _window('END_EFFECTOR_ONLY')
    status = _ready_status('END_EFFECTOR_ONLY')
    status['dual_calibration'] = {'state': 'READY', 'active': False}
    for sample in status['actuators']:
        sample.update(torque_state='ON', hardware_error=0)
    window._update_tool_status(status)
    window._update_mode('MANUAL')
    window.hold_close_button.pressed.emit()
    window._dual_key_jog_tick()
    assert commands == ['JOG_CLOSE', 'JOG_CLOSE']
    if finish == 'release':
        window.hold_close_button.released.emit()
    elif finish == 'close':
        window.close()
    elif finish == 'deactivate':
        window.eventFilter(window, QEvent(QEvent.WindowDeactivate))
    else:
        window._stop_tool()
    assert 'HOLD' in commands
    assert not window.dual_key_jog_timer.isActive()
    before = list(commands)
    window._dual_key_jog_tick()
    assert commands == before
    window.close()


def test_korean_display_preserves_protocol_values():
    from PyQt5.QtWidgets import QLabel, QPushButton, QGroupBox
    import re
    _app, window, commands = _window('END_EFFECTOR_ONLY')
    status = _ready_status('END_EFFECTOR_ONLY')
    status['dual_calibration'] = {'state': 'READY', 'active': False}
    for sample in status['actuators']:
        sample.update(torque_state='ON', hardware_error=0)
    window._update_tool_status(status)
    window._update_mode('MANUAL')
    assert window.hold_open_button.text() == '누르는 동안 열기'
    assert window.hold_close_button.text() == '누르는 동안 닫기'
    assert window.tool_combo.currentData() == 'dual_motor_gripper'
    assert window.tool_combo.currentText() == '2모터 그리퍼'
    requests = []
    window.node.request_mode = requests.append
    window.mode_combo.setCurrentIndex(window.mode_combo.findData('MANUAL'))
    window._request_mode()
    assert requests == ['MANUAL']
    assert window.control_mode == 'MANUAL'
    assert window.fsm_state == 'READY'
    for widget in window.findChildren((QLabel, QPushButton, QGroupBox)):
        text = widget.title() if isinstance(widget, QGroupBox) else widget.text()
        assert not re.search('[A-Za-z]', text), text
    window.close()
