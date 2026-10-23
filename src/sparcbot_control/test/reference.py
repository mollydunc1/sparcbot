"""Helpers for replaying the reference fixture."""

import csv
from collections import defaultdict
from dataclasses import replace
from pathlib import Path

from sparcbot_control.controller_config import WheelControllerConfig

FIXTURE = Path(__file__).parent / 'fixtures' / 'wheel_controller_reference.csv'

# Fixed test values (the ones the fixture was generated with). Independent of
# sparcbot.yaml: measuring your own robot does not affect these tests.
FEEDFORWARD = {
    'left': WheelControllerConfig(
        forward_static=0.0701, forward_velocity_gain=1.739,
        reverse_static=0.062, reverse_velocity_gain=1.7162,
        forward_breakaway=0.15, reverse_breakaway=0.13,
        stopped_velocity_threshold=0.005, max_command=1.0,
        zero_velocity_threshold=0.0001),
    'right': WheelControllerConfig(
        forward_static=0.033, forward_velocity_gain=1.639,
        reverse_static=0.031, reverse_velocity_gain=1.700,
        forward_breakaway=0.10, reverse_breakaway=0.13,
        stopped_velocity_threshold=0.005, max_command=1.0,
        zero_velocity_threshold=0.0001),
}

TOLERANCE = 1e-9

BASE_WIDTH_M = 0.175


def body_command(left_m_s, right_m_s, base_width_m=BASE_WIDTH_M):
    """(v, w) that gives these left/right wheel velocities."""
    return 0.5 * (left_m_s + right_m_s), (right_m_s - left_m_s) / base_width_m


def config_for(wheel, kp, ki, kd):
    return replace(FEEDFORWARD[wheel], kp=kp, ki=ki, kd=kd)


def load_cases():
    """Return {case_id: [row, ...]}; every row is one step for both wheels."""
    cases = defaultdict(list)
    with FIXTURE.open() as f:
        for row in csv.DictReader(f):
            for key, value in row.items():
                if key not in ('case', 'step', 'reset_before'):
                    row[key] = float(value)
            row['step'] = int(row['step'])
            row['reset_before'] = row['reset_before'] == '1'
            cases[int(row['case'])].append(row)
    return dict(cases)
