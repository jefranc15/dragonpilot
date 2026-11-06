"""DNGA R2 exact-controller event probe. OFF-CAR ONLY; never imported by CarController.

This class runs the inherited V5.0 controller unchanged and captures pre-reset
state during real _clear_hydraulic() calls. No commands or brake timings change.
"""
from collections import deque
from selfdrive.car.dnga.longitudinal_kinematic_guard import LongitudinalController as V50Controller


def _finite(x, default=0.0):
  try:
    v = float(x)
    return v if -1e9 < v < 1e9 else default
  except (TypeError, ValueError, OverflowError):
    return default


class LongitudinalController(V50Controller):
  def __init__(self):
    super().__init__()
    self.r2_events = deque(maxlen=20000)
    self.r2_context = None
    self.r2_last_guard = None
    self.r2_replay_only = True
    self.r2_replay_log_ns = -1

  def _update_stop_guard(self, CS, frame, planner_brake, moving_allowed, lead_state):
    guard = super()._update_stop_guard(CS, frame, planner_brake, moving_allowed, lead_state)
    self.r2_last_guard = dict(active=bool(guard.active), entry=bool(guard.entry),
                              predictive=bool(guard.predictive_confirmed),
                              completion=bool(guard.completion))
    return guard

  def _clear_hydraulic(self, frame, reentry=True):
    # _clear_hydraulic resets the latch/counters; capture their true values FIRST.
    before = dict(frame=int(frame), log_mono_ns=int(self.r2_replay_log_ns),
                  reentry=bool(reentry),
                  handoff_active=bool(self.handoff_active),
                  handoff_counter=int(self.handoff_counter),
                  release_counter=int(self.release_counter),
                  coast_release_counter=int(self.coast_release_counter),
                  curve_brake_active=bool(self.curve_brake_active),
                  lead_loss_counter=int(self.lead_loss_counter),
                  apply_brake=_finite(self.apply_brake),
                  sng_armed=bool(self.sng_armed), stop_hold=bool(self.stop_hold),
                  urgent_brake=bool(self.urgent_brake),
                  reentry_deadline=int(self.brake_reentry_frame),
                  context=dict(self.r2_context or {}), guard=dict(self.r2_last_guard or {}))
    self.r2_events.append(dict(event='hydraulic_clear', **before))
    return super()._clear_hydraulic(frame, reentry=reentry)

  def _update_hydraulic(self, CS, frame, session, apply_accel, plan, lead_state):
    was_active = bool(self.brake_active)
    before_counter = int(self.coast_release_counter)
    self.r2_context = dict(log_mono_ns=int(self.r2_replay_log_ns),
      vEgo=_finite(CS.out.vEgo), aEgo=_finite(CS.out.aEgo),
      allowed=bool(session.allowed), plan_fresh=bool(plan.fresh),
      planner_accel=_finite(plan.accel), planner_brake=_finite(plan.brake),
      pid_accel=_finite(apply_accel), lead_relevant=bool(lead_state.relevant),
      lead_status=bool(lead_state.status), closing=_finite(lead_state.closing_speed),
      dRel=_finite(lead_state.distance), ttc=_finite(lead_state.ttc),
      source=str(getattr(plan, 'source', '')),
      turn_speed=_finite(getattr(plan, 'turn_speed', 0.0)),
      curve_brake_active=bool(self.curve_brake_active),
      curve_speed_error=max(0.0, _finite(CS.out.vEgo) - _finite(getattr(plan, 'turn_speed', 0.0))) if plan.curve_active else 0.0,
      engine_rpm=_finite(getattr(CS.out, 'engineRPM', 0.0)),
      pedal_gas=bool(getattr(CS.out, 'gasPressed', False)),
      pedal_brake=bool(getattr(CS.out, 'brakePressed', False)),
      urgent_lead=bool(lead_state.urgent), curve_active=bool(plan.curve_active),
      was_hydraulic=was_active, coast_counter_before=before_counter)
    ret = super()._update_hydraulic(CS, frame, session, apply_accel, plan, lead_state)
    if not was_active and self.brake_active:
      self.r2_events.append(dict(event='hydraulic_entry', frame=int(frame),
        context=dict(self.r2_context), reentry_deadline=int(self.brake_reentry_frame),
        entry_counter=int(self.brake_entry_counter),
        urgent_entry_counter=int(self.urgent_entry_counter),
        guard=dict(self.r2_last_guard or {})))
    self.r2_context = None
    return ret
