"""Joy -> OVR2ROSInputs button mapping."""

import pytest

from quest_webxr_bridge.mapping import JoyMapping, PressLatch, joy_to_inputs


def joy(pressed=(), values=None, thumb=(0.0, 0.0), n=7):
    buttons = [1 if i in pressed else 0 for i in range(n)]
    analog = values or [float(b) for b in buttons]
    return buttons, [0.0, 0.0, thumb[0], thumb[1]] + analog


def test_idle():
    out = joy_to_inputs(*joy())
    assert out == {
        'button_upper': False, 'button_lower': False,
        'thumb_stick_horizontal': 0.0, 'thumb_stick_vertical': 0.0,
        'press_index': 0.0, 'press_middle': 0.0,
    }


def test_upper_face_button_is_b_or_y():
    assert joy_to_inputs(*joy(pressed={5}))['button_upper'] is True
    assert joy_to_inputs(*joy(pressed={5}))['button_lower'] is False


def test_lower_face_button_is_a_or_x():
    assert joy_to_inputs(*joy(pressed={4}))['button_lower'] is True
    assert joy_to_inputs(*joy(pressed={4}))['button_upper'] is False


def test_triggers_are_analog():
    values = [0.3, 0.8, 0, 0, 0, 0, 0]
    out = joy_to_inputs(*joy(values=values))
    assert out['press_index'] == pytest.approx(0.3)
    assert out['press_middle'] == pytest.approx(0.8)


def test_trigger_falls_back_to_pressed_state_without_analog_values():
    buttons = [1, 0, 0, 0, 0, 0]
    out = joy_to_inputs(buttons, [0.0, 0.0, 0.0, 0.0])
    assert out['press_index'] == 1.0


def test_thumbstick_vertical_is_positive_when_pushed_forward():
    # WebXR reports forward as negative y.
    out = joy_to_inputs(*joy(thumb=(0.25, -0.75)))
    assert out['thumb_stick_horizontal'] == pytest.approx(0.25)
    assert out['thumb_stick_vertical'] == pytest.approx(0.75)


def test_custom_mapping_and_short_arrays():
    m = JoyMapping(button_upper_index=3, button_lower_index=9, invert_thumbstick_y=False)
    out = joy_to_inputs([0, 0, 0, 1], [0.0, 0.0, 0.0, -0.5], m)
    assert out['button_upper'] is True
    assert out['button_lower'] is False  # out of range -> not pressed
    assert out['thumb_stick_vertical'] == pytest.approx(-0.5)


def test_press_latch_keeps_short_press_for_one_tick():
    latch = PressLatch()
    latch.observe(joy_to_inputs(*joy(pressed={4})))   # pressed between ticks
    released = joy_to_inputs(*joy())
    latch.observe(released)
    assert latch.apply(released)['button_lower'] is True   # first tick sees the press
    assert latch.apply(released)['button_lower'] is False  # next tick sees the release
