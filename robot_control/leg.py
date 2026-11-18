"""
3-DOF leg: kinematics and a small library of motions.

Joints (all in rad, joint space as seen by Motor):
    q[0] hip_ab   abduction/adduction, rotates the leg plane about the x axis
    q[1] hip      hip pitch, 0 = thigh pointing straight down, + swings foot backward
    q[2] knee     knee pitch relative to thigh, 0 = straight, negative = bent

Frame at the hip: x forward, y left, z up. Foot below the hip has z < 0.
"""

import math
from dataclasses import dataclass, field


@dataclass
class LegGeometry:
    thigh: float = 0.20     # m, hip pitch axis to knee axis
    shank: float = 0.20     # m, knee axis to foot

    @property
    def reach(self):
        return self.thigh + self.shank


def fk(q, g: LegGeometry):
    """Joint angles -> (knee_xyz, foot_xyz)."""
    q0, q1, q2 = q
    kx, kz = -g.thigh * math.sin(q1), -g.thigh * math.cos(q1)
    fx = kx - g.shank * math.sin(q1 + q2)
    fz = kz - g.shank * math.cos(q1 + q2)
    s, c = math.sin(q0), math.cos(q0)
    return (kx, -kz * s, kz * c), (fx, -fz * s, fz * c)


def ik(x, y, z, g: LegGeometry):
    """Foot position -> joint angles (knee bent backward). Unreachable targets are pulled onto the workspace edge."""
    q0 = math.atan2(y, -z)
    zp = -math.hypot(y, z)                      # foot height within the leg plane
    d = math.hypot(x, zp)
    d = max(abs(g.thigh - g.shank) + 1e-4, min(g.reach - 1e-4, d))
    cos_k = (d * d - g.thigh ** 2 - g.shank ** 2) / (2 * g.thigh * g.shank)
    q2 = -math.acos(max(-1.0, min(1.0, cos_k)))
    a = g.thigh + g.shank * math.cos(q2)
    b = g.shank * math.sin(q2)
    q1 = math.atan2(-x, -zp) - math.atan2(b, a)
    return [q0, q1, q2]


def within_reach(xyz, g: LegGeometry, frac=0.98):
    """Pull a foot target in so the knee never has to lock fully straight."""
    d = math.hypot(*xyz)
    limit = frac * g.reach
    return xyz if d <= limit else tuple(v * limit / d for v in xyz)


def min_jerk(s):
    s = max(0.0, min(1.0, s))
    return s * s * s * (10 - 15 * s + 6 * s * s)


def _lerp(a, b, s):
    return [ai + (bi - ai) * s for ai, bi in zip(a, b)]


# --------------------------------------------------------------------------
# Motions. Each one is built from the leg's current joint command, blends in
# smoothly, and exposes joints(t) -> (q, done).
# --------------------------------------------------------------------------

@dataclass
class Param:
    key: str
    label: str
    default: float
    lo: float
    hi: float
    step: float
    unit: str = ""


COMMON = [Param("speed", "Speed", 1.0, 0.25, 2.0, 0.05, "x")]
HEIGHT = Param("height", "Stance height", 0.70, 0.45, 0.90, 0.01, "of reach")


class Motion:
    key = ""
    label = ""
    description = ""
    cyclic = False
    params = []

    def __init__(self, start_q, geom: LegGeometry, **params):
        self.g = geom
        self.start_q = list(start_q)
        defaults = {p.key: p.default for p in COMMON + self.params}
        defaults.update({k: float(v) for k, v in params.items() if k in defaults})
        self.p = defaults
        self.speed = self.p["speed"]
        self.blend = 1.0          # s to move from the start pose onto the motion
        self.setup()

    def setup(self):
        pass

    def stance(self):
        return -self.p.get("height", HEIGHT.default) * self.g.reach

    def joints(self, t):
        """Return (q, done). t is wall time since the motion started."""
        t = t * self.speed
        q, done = self.path(max(0.0, t - self.blend))
        if t < self.blend:
            q = _lerp(self.start_q, q, min_jerk(t / self.blend))
            done = False
        return q, done

    def path(self, t):
        raise NotImplementedError


class KeyframeMotion(Motion):
    """Joint-space min-jerk moves through foot-space keyframes [(xyz, seconds), ...]."""

    def keyframes(self):
        raise NotImplementedError

    def setup(self):
        frames = self.keyframes()
        self.blend = frames[0][1]
        self.poses = [ik(*within_reach(xyz, self.g), self.g) for xyz, _ in frames]
        self.durations = [d for _, d in frames]

    def path(self, t):
        prev = self.poses[0]
        for pose, dur in zip(self.poses[1:], self.durations[1:]):
            if t < dur:
                return _lerp(prev, pose, min_jerk(t / dur)), False
            t -= dur
            prev = pose
        return list(self.poses[-1]), True


class Stand(KeyframeMotion):
    key, label = "stand", "Stand"
    description = "Move the foot straight under the hip at stance height."
    params = [HEIGHT]

    def keyframes(self):
        return [((0.0, 0.0, self.stance()), 1.5)]


class Extend(KeyframeMotion):
    key, label = "extend", "Extend"
    description = "Push the foot down to near-full leg extension, then return."
    params = [HEIGHT, Param("depth", "Extension", 0.95, 0.75, 0.98, 0.01, "of reach"),
              Param("hold", "Hold time", 1.0, 0.0, 3.0, 0.1, "s")]

    def keyframes(self):
        down = -self.p["depth"] * self.g.reach
        return [((0.0, 0.0, self.stance()), 1.2),
                ((0.0, 0.0, down), 1.0),
                ((0.0, 0.0, down), self.p["hold"] + 1e-3),
                ((0.0, 0.0, self.stance()), 1.0)]


class Tuck(KeyframeMotion):
    key, label = "tuck", "Tuck"
    description = "Fold the leg up tight under the hip."
    params = [Param("tuck", "Tuck height", 0.40, 0.30, 0.60, 0.01, "of reach")]

    def keyframes(self):
        return [((-0.02, 0.0, -self.p["tuck"] * self.g.reach), 1.5)]


class Kick(KeyframeMotion):
    key, label = "kick", "Kick"
    description = "Wind up behind, snap forward, settle back to stance."
    params = [HEIGHT, Param("reach", "Kick reach", 0.55, 0.2, 0.8, 0.01, "of reach")]

    def keyframes(self):
        L, z = self.g.reach, self.stance()
        return [((0.0, 0.0, z), 1.0),
                ((-0.25 * L, 0.0, 0.85 * z), 0.8),
                ((self.p["reach"] * L, 0.0, 0.75 * z), 0.3),
                ((self.p["reach"] * L, 0.0, 0.75 * z), 0.3),
                ((0.0, 0.0, z), 0.9)]


class CyclicMotion(Motion):
    cyclic = True

    def foot(self, phase):
        raise NotImplementedError

    def setup(self):
        self.period = self.p["period"]
        self.blend = 1.0

    def path(self, t):
        return ik(*within_reach(self.foot((t / self.period) % 1.0), self.g), self.g), False


PERIOD = Param("period", "Cycle time", 1.6, 0.6, 4.0, 0.1, "s")


class Crawl(CyclicMotion):
    key, label = "crawl", "Crawl"
    description = "Walking foot path: slow stance stroke along the ground, quick arcing swing forward."
    params = [HEIGHT, PERIOD,
              Param("stride", "Stride", 0.30, 0.0, 0.6, 0.01, "of reach"),
              Param("lift", "Step height", 0.15, 0.03, 0.35, 0.01, "of reach"),
              Param("duty", "Ground time", 0.75, 0.5, 0.9, 0.01, "of cycle")]

    def foot(self, phase):
        L, z = self.g.reach, self.stance()
        half = self.p["stride"] * L / 2
        duty = self.p["duty"]
        if phase < duty:                                   # stance: front -> back
            s = phase / duty
            return (half - 2 * half * s, 0.0, z)
        s = (phase - duty) / (1 - duty)                    # swing: back -> front, lifted
        x = -half + 2 * half * (0.5 - 0.5 * math.cos(math.pi * s))
        return (x, 0.0, z + self.p["lift"] * L * math.sin(math.pi * s))


class Step(Crawl):
    key, label = "step", "Step in place"
    description = "Lift and plant the foot without moving forward."
    params = [HEIGHT, PERIOD,
              Param("lift", "Step height", 0.15, 0.03, 0.35, 0.01, "of reach"),
              Param("duty", "Ground time", 0.6, 0.5, 0.9, 0.01, "of cycle")]

    def setup(self):
        self.p["stride"] = 0.0
        super().setup()


class Circle(CyclicMotion):
    key, label = "circle", "Circle"
    description = "Trace a circle with the foot in the side view."
    params = [HEIGHT, PERIOD, Param("radius", "Radius", 0.15, 0.03, 0.25, 0.01, "of reach")]

    def foot(self, phase):
        # keep the bottom of the circle short of full extension
        r = max(0.0, min(self.p["radius"], 0.9 - self.p["height"])) * self.g.reach
        a = 2 * math.pi * phase
        return (r * math.sin(a), 0.0, self.stance() + r * math.cos(a))


class Sway(CyclicMotion):
    key, label = "sway", "Sway"
    description = "Swing the foot side to side with the hip abduction joint."
    params = [HEIGHT, PERIOD, Param("width", "Width", 0.25, 0.05, 0.4, 0.01, "of reach")]

    def foot(self, phase):
        return (0.0, self.p["width"] * self.g.reach * math.sin(2 * math.pi * phase), self.stance())


MOTIONS = {m.key: m for m in [Stand, Extend, Tuck, Crawl, Step, Circle, Kick, Sway]}


def motion_catalog():
    return [{
        "key": m.key, "label": m.label, "description": m.description, "cyclic": m.cyclic,
        "params": [p.__dict__ for p in COMMON + m.params],
    } for m in MOTIONS.values()]
