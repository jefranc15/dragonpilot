"""DNGA V5.0-R1 experimental pure coast and handoff release policy.

Does not generate CAN or change hydraulic magnitude. Integrated live changes
remain DISABLED by default until controller replay and controlled validation.
"""

COAST_PREP_MIN_SPEED = 8.0        # m/s
COAST_PREP_MIN_CLOSING = 3.0     # m/s
COAST_PREP_MAX_TTC = 12.0        # s
COAST_PREP_MAX_DISTANCE = 100.0  # m
COAST_PREP_CONFIRM = 3           # 20 Hz cycles ~=0.15 s

HANDOFF_RECHECK_MIN_BRAKE = 0.08  # m/s^2
HANDOFF_RECHECK_PID_MAX = -0.05   # m/s^2


def should_prepare_coast(*, allowed, moving_speed, plan_fresh, lead_relevant,
                         lead_status, distance, closing, ttc, urgent, curve,
                         hydraulic, release, hold):
  return bool(
    allowed and plan_fresh and lead_relevant and lead_status
    and (not urgent) and (not curve)
    and (not hydraulic) and (not release) and (not hold)
    and moving_speed >= COAST_PREP_MIN_SPEED
    and max(8.0, moving_speed * 0.6) <= distance <= COAST_PREP_MAX_DISTANCE
    and closing >= COAST_PREP_MIN_CLOSING
    and 0.0 < ttc <= COAST_PREP_MAX_TTC
  )


def should_abort_handoff(*, allowed, braking, curve, hold, plan_fresh,
                         lead_relevant, lead_status, planner_brake, pid_accel):
  """Abort a latched coast release when fresh lead/PID demand turns negative.

  Does not turn ON brakes, raise magnitude, or modify emergency handling.
  Only prevents an old positive-PID release latch from overruling later
  negative current demand while braking is still active.
  """
  return bool(
    allowed and braking and not curve and not hold
    and plan_fresh and lead_relevant and lead_status
    and planner_brake >= HANDOFF_RECHECK_MIN_BRAKE
    and pid_accel <= HANDOFF_RECHECK_PID_MAX
  )

class CoastPreparationLatch:
  def __init__(self):
    self.count = 0
    self.active = False

  def update(self, eligible):
    self.count = min(self.count + 1, COAST_PREP_CONFIRM) if eligible else 0
    self.active = self.count >= COAST_PREP_CONFIRM
    return self.active

  def reset(self):
    self.count = 0
    self.active = False

