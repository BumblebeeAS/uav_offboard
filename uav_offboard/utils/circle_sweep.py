"""Time-parametrized horizontal sweep, including centre entry/return."""

import math
import numpy as np

TRANSITION_DURATION = 0.5  # Minimum for small circles.
TRANSITION_SPEED = 0.2  # Average radial entry/return speed, m/s.


def sweep_transition_duration(radius):
    """Scale radial transitions so large circles do not cause fast lateral jumps."""
    return max(TRANSITION_DURATION, radius / TRANSITION_SPEED)


def sweep_total_duration(radius, duration, start_on_circle=False):
    return duration if start_on_circle else duration + 2.0 * sweep_transition_duration(radius)


def sweep_offset(elapsed, radius, duration, start_on_circle=False):
    """Start/end at centre; one full circle at constant altitude.

    Radial transitions average at most 0.2 m/s (smoothstep peak 0.3 m/s),
    with a minimum of half a second. Duration covers the revolution itself.
    """
    if start_on_circle:
        # Clamp at the starting edge after one revolution while settling there.
        angle = 2.0 * math.pi * min(1.0, max(0.0, elapsed / duration))
        return np.array([radius * math.cos(angle), radius * math.sin(angle), 0.0])
    transition = sweep_transition_duration(radius)
    if elapsed <= 0.0:
        return np.zeros(3)
    if elapsed < transition:
        t = elapsed / transition
        return np.array([radius * t * t * (3.0 - 2.0 * t), 0.0, 0.0])
    elapsed -= transition
    if elapsed < duration:
        angle = 2.0 * math.pi * elapsed / duration
        return np.array([radius * math.cos(angle), radius * math.sin(angle), 0.0])
    t = min(1.0, (elapsed - duration) / transition)
    return np.array([radius * (1.0 - t * t * (3.0 - 2.0 * t)), 0.0, 0.0])
