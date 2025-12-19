import numpy as np
import pyray as rl
from opendbc.car import ACCELERATION_DUE_TO_GRAVITY
from openpilot.selfdrive.ui.mici.onroad import blend_colors
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.shader_polygon import draw_polygon, Gradient
from openpilot.system.ui.widgets import Widget
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.ui.mici.onroad.torque_bar import arc_bar_pts

# TODO: arc_bar_pts doesn't consider rounded end caps part of the angle span
TORQUE_ANGLE_SPAN = 12.7

DEBUG = False

SCALE = 2.5


class TorqueBar(Widget):
  def __init__(self, demo: bool = False):
    super().__init__()
    self._demo = demo
    self._torque_filter = FirstOrderFilter(0, 0.1, 1 / gui_app.target_fps)
    self._torque_line_alpha_filter = FirstOrderFilter(0.0, 0.1, 1 / gui_app.target_fps)

  def update_filter(self, value: float):
    """Update the torque filter value (for demo mode)."""
    self._torque_filter.update(value)

  def _update_state(self):
    if self._demo:
      return

    # torque line
    if ui_state.sm['controlsState'].lateralControlState.which() == 'angleState':
      controls_state = ui_state.sm['controlsState']
      car_state = ui_state.sm['carState']
      live_parameters = ui_state.sm['liveParameters']
      lateral_acceleration = controls_state.curvature * car_state.vEgo ** 2 - live_parameters.roll * ACCELERATION_DUE_TO_GRAVITY
      # TODO: pull from carparams
      max_lateral_acceleration = 3

      # from selfdrived
      actual_lateral_accel = controls_state.curvature * car_state.vEgo ** 2
      desired_lateral_accel = controls_state.desiredCurvature * car_state.vEgo ** 2
      accel_diff = (desired_lateral_accel - actual_lateral_accel)

      self._torque_filter.update(min(max(lateral_acceleration / max_lateral_acceleration + accel_diff, -1), 1))
    else:
      self._torque_filter.update(-ui_state.sm['carOutput'].actuatorsOutput.torque)

  def _render(self, rect: rl.Rectangle) -> None:
    # adjust y pos with torque
    torque_line_offset = np.interp(abs(self._torque_filter.x), [0.5, 1], [22 * SCALE, 26 * SCALE])
    torque_line_height = np.interp(abs(self._torque_filter.x), [0.5, 1], [14 * SCALE, 56 * SCALE])

    # animate alpha and angle span
    if not self._demo:
      self._torque_line_alpha_filter.update(ui_state.status != UIStatus.DISENGAGED)
    else:
      self._torque_line_alpha_filter.update(1.0)

    torque_line_bg_alpha = np.interp(abs(self._torque_filter.x), [0.5, 1.0], [0.25, 0.5])
    torque_line_bg_color = rl.Color(255, 255, 255, int(255 * torque_line_bg_alpha * self._torque_line_alpha_filter.x))
    if ui_state.status != UIStatus.ENGAGED and not self._demo:
      torque_line_bg_color = rl.Color(255, 255, 255, int(255 * 0.15 * self._torque_line_alpha_filter.x))

    # draw curved line polygon torque bar
    torque_line_radius = 1200 * SCALE
    top_angle = -90
    torque_bg_angle_span = self._torque_line_alpha_filter.x * TORQUE_ANGLE_SPAN
    torque_start_angle = top_angle - torque_bg_angle_span / 2
    torque_end_angle = top_angle + torque_bg_angle_span / 2
    # centerline radius & center
    mid_r = torque_line_radius + torque_line_height / 2

    cx = rect.x + rect.width / 2 + (8 * SCALE)
    cy = rect.y + rect.height + torque_line_radius - torque_line_offset

    # SCALED: pass cap_radius explicitly so the corners round properly
    scaled_cap_radius = 7 * SCALE

    # draw bg torque indicator line
    bg_pts = arc_bar_pts(cx, cy, mid_r, torque_line_height, torque_start_angle, torque_end_angle,
                         cap_radius=scaled_cap_radius)
    draw_polygon(rect, bg_pts, color=torque_line_bg_color)

    # draw torque indicator line
    a0s = top_angle
    a1s = a0s + torque_bg_angle_span / 2 * self._torque_filter.x
    sl_pts = arc_bar_pts(cx, cy, mid_r, torque_line_height, a0s, a1s,
                         cap_radius=scaled_cap_radius)

    # draw beautiful gradient from center to 65% of the bg torque bar width
    start_grad_pt = cx / rect.width
    if self._torque_filter.x < 0:
      end_grad_pt = (cx * (1 - 0.65) + (min(bg_pts[:, 0]) * 0.65)) / rect.width
    else:
      end_grad_pt = (cx * (1 - 0.65) + (max(bg_pts[:, 0]) * 0.65)) / rect.width

    # fade to orange as we approach max torque
    start_color = blend_colors(
      rl.Color(255, 255, 255, int(255 * 0.9 * self._torque_line_alpha_filter.x)),
      rl.Color(255, 200, 0, int(255 * self._torque_line_alpha_filter.x)),  # yellow
      max(0, abs(self._torque_filter.x) - 0.75) * 4,
    )
    end_color = blend_colors(
      rl.Color(255, 255, 255, int(255 * 0.9 * self._torque_line_alpha_filter.x)),
      rl.Color(255, 115, 0, int(255 * self._torque_line_alpha_filter.x)),  # orange
      max(0, abs(self._torque_filter.x) - 0.75) * 4,
    )

    if ui_state.status != UIStatus.ENGAGED and not self._demo:
      start_color = end_color = rl.Color(255, 255, 255, int(255 * 0.35 * self._torque_line_alpha_filter.x))

    gradient = Gradient(
      start=(start_grad_pt, 0),
      end=(end_grad_pt, 0),
      colors=[
        start_color,
        end_color,
      ],
      stops=[0.0, 1.0],
    )

    draw_polygon(rect, sl_pts, gradient=gradient)

    # draw center torque bar dot
    if abs(self._torque_filter.x) < 0.5:
      dot_y = rect.y + rect.height - torque_line_offset - torque_line_height / 2
      rl.draw_circle(int(cx), int(dot_y), int(10 * SCALE) // 2,
                     rl.Color(182, 182, 182, int(255 * 0.9 * self._torque_line_alpha_filter.x)))
