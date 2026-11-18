"""
Robot motor layout. Edit this when the robot's joints are decided.

Each motor on the same RS-485 adapter needs a unique ID (0-14). New motors ship
as ID 0, so change IDs one motor at a time with build/changeID or the
motor_tools/changeid tool before putting several on one bus.

Limits below are conservative bring-up values; raise them once things work.
"""

from leg import LegGeometry
from motors import MotorConfig

# One 3-DOF leg. Joint conventions are documented in leg.py:
#   hip_ab 0 = leg vertical in the front view, hip/knee 0 = leg straight down.
# If a joint moves the wrong way on the real leg, flip its direction to -1.
MOTORS = [
    MotorConfig(name="hip_ab", id=9, port="/dev/ttyUSB3", q_min=-0.6, q_max=0.6),
    MotorConfig(name="hip", id=10, port="/dev/ttyUSB3", q_min=-1.2, q_max=1.8),
    MotorConfig(name="knee", id=11, port="/dev/ttyUSB3", q_min=-2.6, q_max=-0.15),
]

# Measure these on the real leg (meters, joint axis to joint axis / foot tip).
LEG = LegGeometry(thigh=0.20, shank=0.20)
