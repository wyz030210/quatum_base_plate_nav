"""Graphical regression for loaded defaults and debounced widget edits."""
import os
import time

import pytest

from nav2_mission.coverage_control_gui import CoverageWindow
import test_coverage_controls_ros as controls_test

controls = controls_test.controls


@pytest.mark.skipif(not os.environ.get('DISPLAY'), reason='requires an X11 display')
def test_widgets_show_loaded_values_and_apply_edits_before_planning(controls):
    import tkinter

    gui, planner, _, boundary, operation, _ = controls
    try:
        window = CoverageWindow(gui)
    except tkinter.TclError as exc:
        pytest.skip(str(exc))
    window.root.withdraw()

    def pump(predicate, timeout=8):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            window.root.update()
            time.sleep(0.02)
        assert predicate()

    try:
        pump(lambda: window.width.get() == '0.5' and window.angle.get() == '0.75')
        window.width.set('0.55')
        window.angle.set('0.2')
        pump(lambda: gui.state['planner'] == {
            'operation_width': 0.55, 'default_swath_angle': 0.2})
        controls_test.rectangle(operation)
        window.width.set('0.52')
        window.request_action('preview')  # Do not wait for the 500ms debounce.
        pump(lambda: '规划成功' in window.notice.get())
        assert planner.get_parameter('operation_width').value == 0.52
        assert planner.get_parameter('default_swath_angle').value == 0.2
        window.width.set('0')
        window.request_action('preview')
        assert len(boundary['goals']) == 1
        assert gui.state['planner']['operation_width'] == 0.52
    finally:
        if window._apply_after is not None:
            window.root.after_cancel(window._apply_after)
        window.root.destroy()
