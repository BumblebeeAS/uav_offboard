"""Arc-length-parametrized outward spiral with bounded speed and acceleration."""
import math

import numpy as np


class SpiralPath:
    """Freeze a horizontal path; accelerate gently and slow through tight turns."""

    def __init__(self, radius, turn_spacing, speed, acceleration=0.4):
        if not all(math.isfinite(v) and v > 0
                   for v in (radius, turn_spacing, speed, acceleration)):
            raise ValueError('Spiral parameters must be finite and positive')
        # A radial pitch larger than the diameter is not a useful spiral search.
        if turn_spacing > 2 * radius:
            raise ValueError('Spiral turn spacing must not exceed its diameter')
        b = turn_spacing / (2 * math.pi)
        end = radius / b
        theta = np.linspace(0., end, max(1001, int(end * 200) + 1))
        r = b * theta
        self.points = np.column_stack((r * np.cos(theta), r * np.sin(theta), np.zeros_like(r)))
        ds = np.linalg.norm(np.diff(self.points, axis=0), axis=1)
        # Normal acceleration v^2 * curvature <= acceleration, including centre.
        curvature = (r * r + 2 * b * b) / (r * r + b * b) ** 1.5
        v = np.minimum(speed, np.sqrt(acceleration / curvature))
        v[0] = v[-1] = 0.
        for i in range(1, len(v)):
            v[i] = min(v[i], math.sqrt(v[i-1]**2 + 2 * acceleration * ds[i-1]))
        for i in range(len(v)-2, -1, -1):
            v[i] = min(v[i], math.sqrt(v[i+1]**2 + 2 * acceleration * ds[i]))
        self.speeds, self.distances = v, ds
        self.times = np.r_[0., np.cumsum(2 * ds / (v[:-1] + v[1:]))]
        self.duration = float(self.times[-1])

    def offset(self, elapsed):
        if elapsed <= 0:
            return self.points[0].copy()
        if elapsed >= self.duration:
            return self.points[-1].copy()
        i = int(np.searchsorted(self.times, elapsed, side='right') - 1)
        dt = elapsed - self.times[i]
        segment_time = self.times[i+1] - self.times[i]
        acceleration = (self.speeds[i+1] - self.speeds[i]) / segment_time
        distance = self.speeds[i] * dt + .5 * acceleration * dt * dt
        fraction = distance / self.distances[i]
        return self.points[i] + fraction * (self.points[i+1] - self.points[i])
