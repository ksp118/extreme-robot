"""Real ROS callbacks and Qt button clicks with the in-memory bridge backend."""
import os
import time
from pathlib import Path

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')


def test_mock_runtime_spur_manual_enable_open_close_jog(monkeypatch, tmp_path):
    import rclpy
    import yaml
    from rclpy.executors import SingleThreadedExecutor
    from PyQt5.QtWidgets import QApplication, QPushButton
    from dynamixel_sdk import PortHandler
    from dynamixel_control.moveit_dynamixel_bridge import MoveItDynamixelBridge
    from robot_manual_gui.ros_interface import ManualGuiNode, GuiSignals
    from robot_manual_gui.main_window import ManualMainWindow
    from robot_manual_gui.korean_text import ko

    def forbidden(*args, **kwargs):
        raise AssertionError('mock test attempted physical serial access')
    monkeypatch.setattr(PortHandler, 'openPort', forbidden)
    source_profile = Path(__file__).parents[2] / 'dynamixel_control/config/tool_profiles.yaml'
    profiles = yaml.safe_load(source_profile.read_text())
    # A mock-only calibrated fixture must not certify the physical tool.
    profiles['tool_profiles']['spur_1motor_gripper']['calibrated'] = True
    profile_file = tmp_path / 'mock_tool_profiles.yaml'
    profile_file.write_text(yaml.safe_dump(profiles))
    rclpy.init(args=['--ros-args', '-p', 'mock_mode:=true', '-p', 'read_only:=false',
                     '-p', 'tool_type:=dual_motor_gripper', '-p',
                     'control_scope:=END_EFFECTOR_ONLY', '-p',
                     f'tool_profile_file:={profile_file}'])
    app = QApplication.instance() or QApplication([])
    bridge = MoveItDynamixelBridge()
    signals = GuiSignals()
    gui = ManualGuiNode(signals)
    window = ManualMainWindow(gui, signals, bridge.tool_profile, mock_mode=True)
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(gui)

    def wait_for(predicate):
        until = time.monotonic() + 5
        while time.monotonic() < until:
            executor.spin_once(timeout_sec=0.02)
            app.processEvents()
            if predicate():
                return
        raise AssertionError(f'timed out: tool={gui.selected_tool}, FSM={window.fsm_state}, '
                             f'mode={window.control_mode}, status={window.tool_status}')

    try:
        wait_for(lambda: window.fsm_state == 'READY')
        window.tool_combo.setCurrentIndex(window.tool_combo.findData('spur_1motor_gripper'))
        next(b for b in window.findChildren(QPushButton)
             if b.text() == ko('REQUEST TOOL CHANGE')).click()
        wait_for(lambda: gui.selected_tool == 'spur_1motor_gripper' and window.fsm_state == 'READY')
        assert type(bridge.tool_fsm).__name__ == 'SingleMotorGripperFSM'
        assert bridge.tool_ids == gui.actuator_ids == [5]
        assert bridge.read_torque(5) == 0
        # Exercise the real hardware-mode GUI ownership gate too; only bridge
        # register observations are mocked, never the READY permission check.
        window.mock_mode = False
        window.mode_combo.setCurrentIndex(window.mode_combo.findData('MANUAL'))
        next(b for b in window.findChildren(QPushButton)
             if b.text() == ko('REQUEST MODE')).click()
        wait_for(lambda: window.control_mode == gui.control_mode == bridge.control_mode == 'MANUAL')
        window.mock_mode = True
        # Reuse the existing explicit session-start requirement for ENABLE.
        wait_for(lambda: window.start_cal.isEnabled())
        window.start_cal.click()
        wait_for(lambda: window.spur_enable.isEnabled())
        window.spur_enable.click()
        wait_for(lambda: bridge.read_torque(5) == 1 and window.spur_torque_state == 'ON')
        assert window.open_button.isEnabled()
        window.open_button.click()
        wait_for(lambda: window.fsm_state == 'OPEN')
        assert bridge.read_position(5) == bridge.tool_profile['open_tick']
        assert window.close_button.isEnabled()
        window.close_button.click()
        wait_for(lambda: window.fsm_state == 'CLOSED')
        assert bridge.read_position(5) == bridge.tool_profile['close_tick']
        before = bridge.read_position(5)
        assert window.motor_minus_half.isEnabled()
        window.motor_minus_half.click()
        wait_for(lambda: bridge.read_position(5) == before - 6)
        window.jog_open.click()
        wait_for(lambda: bridge.read_position(5) == before)
        assert set(bridge._tool_samples) == {5}
        assert window.dual_enable.isHidden()
    finally:
        window.close()
        executor.shutdown()
        gui.destroy_node()
        bridge.destroy_node()
        rclpy.shutdown()
