#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Offline geometric diagnostics. No range rejection or navigation decisions."""
import itertools

import numpy as np


def validate(anchors, ranges):
    anchors = np.asarray(anchors, dtype=float)
    ranges = np.asarray(ranges, dtype=float)
    if anchors.ndim != 2 or anchors.shape[1] != 3 or ranges.shape != (len(anchors),):
        raise ValueError('expected Nx3 anchors and N ranges')
    if not np.all(np.isfinite(anchors)) or not np.all(np.isfinite(ranges)) or np.any(ranges <= 0):
        raise ValueError('nonfinite coordinates or invalid ranges')
    return anchors, ranges


def sphere_intersections(anchors, ranges):
    """Return both intersections of three spheres, or no exact real solution.

    Exact geometry only: no noise inflation, height constraint or preferred side.
    Roundoff tolerances are scaled to local baseline/range magnitudes.
    """
    anchors, ranges = validate(anchors, ranges)
    if len(anchors) != 3:
        raise ValueError('three spheres required')
    delta = anchors[1] - anchors[0]
    length = np.linalg.norm(delta)
    scale = max(1.0, float(np.max(ranges)), float(np.max(np.linalg.norm(anchors - anchors[0], axis=1))))
    if length <= 1.e-10 * scale:
        return []
    ex = delta / length
    offset = anchors[2] - anchors[0]
    along = np.dot(ex, offset)
    orthogonal = offset - along * ex
    width = np.linalg.norm(orthogonal)
    if width <= 1.e-10 * scale:
        return []
    ey = orthogonal / width
    ez = np.cross(ex, ey)
    x = (ranges[0]**2 - ranges[1]**2 + length**2) / (2 * length)
    y = (ranges[0]**2 - ranges[2]**2 + along**2 + width**2 - 2 * along * x) / (2 * width)
    z2 = ranges[0]**2 - x*x - y*y
    if z2 < -1.e-10 * scale*scale:
        return []
    center = anchors[0] + x * ex + y * ey
    z = np.sqrt(max(0.0, z2))
    return [center] if z == 0 else [center + z * ez, center - z * ez]


def leave_one_out(anchors, ranges):
    """Enumerate hypotheses, never pick the anchor with the largest residual.

    With four ranges in 3D, removing one leaves three sphere constraints that
    can fit exactly on either side of their plane. Multiple exact hypotheses
    cannot identify a faulty anchor, even if its error is assumed positive.
    """
    anchors, ranges = validate(anchors, ranges)
    if len(anchors) != 4:
        raise ValueError('this diagnostic enumerates four-anchor ambiguity only')
    result = []
    for omitted in range(4):
        keep = np.arange(4) != omitted
        candidates = []
        for position in sphere_intersections(anchors[keep], ranges[keep]):
            residual = np.linalg.norm(position - anchors[keep], axis=1) - ranges[keep]
            candidates.append({'position': position.tolist(), 'kept_rms_m': float(np.sqrt(np.mean(residual**2))),
                               'held_out_m': float(ranges[omitted] - np.linalg.norm(position - anchors[omitted]))})
        result.append(candidates)
    return result


def pairwise_violation(anchors, ranges):
    """Necessary triangle bounds only. Zero does not imply a consistent set."""
    anchors, ranges = validate(anchors, ranges)
    return max([0.0] + [max(abs(ranges[i] - ranges[j]) - np.linalg.norm(anchors[i] - anchors[j]),
                            np.linalg.norm(anchors[i] - anchors[j]) - ranges[i] - ranges[j])
                        for i, j in itertools.combinations(range(len(anchors)), 2)])


def plane(anchors):
    anchors = np.asarray(anchors, dtype=float)
    center = anchors.mean(axis=0)
    _, singular, vectors = np.linalg.svd(anchors - center)
    normal = vectors[-1]
    if normal[2] < 0:
        normal = -normal
    return center, normal, singular
