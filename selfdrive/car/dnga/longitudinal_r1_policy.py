"""DNGA V5.0-R1 experimental pure coast and lead-blip policy.

Does not generate CAN or change hydraulic magnitude. Integrated live changes
remain DISABLED by default until controller replay and controlled validation.
"""

COAST_PREP_MIN_SPEED = 8.0        # m/s
COAST_PREP_MIN_CLOSING = 3.0     # m/s
COAST_PREP_MAX_TTC = 12.0        # s
COAST_PREP_MAX_DISTANCE = 100.0  # m
COAST_PREP_CONFIRM = 3           # 20 Hz cycles ~=0.15 s

LEAD_BLIP_MIN_CLOSING = 0.5
LEAD_BLIP_MAX_TTC = 15.0
LEAD_BLIP_MAX_DISTANCE = 90.0
LEAD_BLIP_MIN_PLANNER_BRAKE = 0.08
LEAD_BLIP_MAX_RADAR_AGE_FRAMES = 15  # 100 Hz = 0.15 s
LEAD_BLIP_GRACE_CYCLES = 2           # 20 Hz = 0.10 s


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


def should_bridge_lead_blip(*, allowed, braking, curve, plan_fresh, plan_has_lead,
                            radar_fresh, lead_status, distance, closing, ttc,
                            planner_brake):
  return bool(
    allowed and braking and (not curve)
    and plan_fresh and (not plan_has_lead)
    and radar_fresh and lead_status
    and 5.0 <= distance <= LEAD_BLIP_MAX_DISTANCE
    and closing >= LEAD_BLIP_MIN_CLOSING
    and 0.0 < ttc <= LEAD_BLIP_MAX_TTC
    and planner_brake >= LEAD_BLIP_MIN_PLANNER_BRAKE
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


class LeadBlipGrace:
  def __init__(self):
    self.armed = False
    self.missing = 0

  def update(self, *, braking, plan_has_lead, trusted_lead, eligible):
    if not braking:
      self.armed = False
      self.missing = 0
      return False
    if plan_has_lead:
      # Only a newly re-trusted lead may rearm the grace budget.
      self.armed = bool(trusted_lead)
      self.missing = 0
      return False
    if not eligible or not self.armed:
      self.armed = False
      self.missing = 0
      return False
    self.missing += 1
    if self.missing <= LEAD_BLIP_GRACE_CYCLES:
      return True
    self.armed = False
    self.missing = 0
    return False
