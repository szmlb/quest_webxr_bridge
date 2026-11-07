"""Joy to Quest2ROS inputs mapping, independent of ROS message classes.

The browser publishes sensor_msgs/Joy with this layout (see web/js/core.js):

* ``buttons[i]``: 1 if xr-standard button ``i`` is pressed, else 0.
* ``axes[0:4]``: xr-standard axes (touchpad x, touchpad y, thumbstick x,
  thumbstick y). Browsers that report only ``[x, y]`` for the thumbstick are
  normalized into slots 2 and 3.
* ``axes[4 + i]``: analog value (0..1) of xr-standard button ``i``.

xr-standard button order for Touch controllers:
0 trigger, 1 squeeze (grip), 2 touchpad (absent on Touch), 3 thumbstick press,
4 A / X, 5 B / Y, 6 thumbrest (if reported).
"""

from dataclasses import dataclass, field
from typing import Dict, Sequence

JOY_AXES_SLOTS = 4


@dataclass
class JoyMapping:
    """Indices into the Joy layout. Defaults follow the xr-standard mapping."""

    button_upper_index: int = 5   # B (right) / Y (left)
    button_lower_index: int = 4   # A (right) / X (left)
    index_trigger_index: int = 0  # trigger -> press_index
    middle_trigger_index: int = 1  # squeeze -> press_middle
    thumbstick_x_axis: int = 2
    thumbstick_y_axis: int = 3
    invert_thumbstick_y: bool = True  # WebXR: forward is negative; Quest2ROS: forward is positive


def _get(seq: Sequence, i: int, default=0):
    return seq[i] if 0 <= i < len(seq) else default


def button_value(buttons: Sequence[int], axes: Sequence[float], index: int) -> float:
    """Analog value of a button, falling back to its pressed state."""
    analog_slot = JOY_AXES_SLOTS + index
    if 0 <= index and analog_slot < len(axes):
        return float(axes[analog_slot])
    return float(_get(buttons, index, 0))


def joy_to_inputs(buttons: Sequence[int], axes: Sequence[float],
                  mapping: JoyMapping = JoyMapping()) -> Dict[str, object]:
    """Return the OVR2ROSInputs field values for one Joy sample."""
    y = float(_get(axes, mapping.thumbstick_y_axis, 0.0))
    return {
        'button_upper': bool(_get(buttons, mapping.button_upper_index, 0)),
        'button_lower': bool(_get(buttons, mapping.button_lower_index, 0)),
        'thumb_stick_horizontal': float(_get(axes, mapping.thumbstick_x_axis, 0.0)),
        'thumb_stick_vertical': -y if mapping.invert_thumbstick_y else y,
        'press_index': button_value(buttons, axes, mapping.index_trigger_index),
        'press_middle': button_value(buttons, axes, mapping.middle_trigger_index),
    }


@dataclass
class PressLatch:
    """Keeps short presses visible across resampling.

    The bridge republishes at a fixed rate that is lower than the headset frame
    rate. Quest2ROS2 acts on rising edges of the buttons, so a press that starts
    and ends between two output ticks would be lost. A latched button reads as
    pressed for at least one output tick.
    """

    _latched: Dict[str, bool] = field(default_factory=dict)

    def observe(self, inputs: Dict[str, object]) -> None:
        for key in ('button_upper', 'button_lower'):
            if inputs.get(key):
                self._latched[key] = True

    def apply(self, inputs: Dict[str, object]) -> Dict[str, object]:
        out = dict(inputs)
        for key, pressed in self._latched.items():
            out[key] = bool(out.get(key)) or pressed
        self._latched.clear()
        return out
