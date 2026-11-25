"""
Jedinični testovi za matematičke funkcije u algorithm.py.
Pokreću se putem `pytest` unutar ili izvan ROS okruženja.
"""
import math
import numpy as np
import pytest

from kh_disparity_extender.algorithm import (
    calculate_speed,
    detect_and_extend_disparities,
    find_furthest_direction,
    preprocess_scan,
)


def test_preprocess_scan():
    # 180 zraka od -90° do +90°
    angles = np.linspace(-math.pi / 2, math.pi / 2, 180)
    ranges = np.ones(180) * 5.0
    # Ubaci nan i inf vrijednosti
    ranges[10] = np.nan
    ranges[20] = np.inf
    ranges[30] = 25.0  # veće od max_range

    proc_ranges, proc_angles = preprocess_scan(
        ranges, angles, fov_deg=120.0, max_range=10.0, filter_window_size=1
    )

    # 120 stupnjeva od 180 stupnjeva = 120 zraka
    assert len(proc_ranges) == 120
    assert np.all(proc_ranges <= 10.0)
    assert not np.any(np.isnan(proc_ranges))
    assert not np.any(np.isinf(proc_ranges))


def test_disparity_extension_forward():
    """
    Testira skok s bliskog zida na daleki prostor (npr. zrake 0..19 su 2m, a 20..49 su 8m).
    Daleke zrake nakon indeksa 20 moraju biti skraćene na 2m za širinu auta.
    """
    ranges = np.ones(50) * 8.0
    ranges[:20] = 2.0  # prepreka na 2 metra
    angle_increment = math.radians(0.25)

    car_width = 0.30
    safety_margin = 0.10
    clearance = 0.25  # 0.15 + 0.10
    # Broj zraka: ceil(0.25 / (2.0 * 0.00436)) = ceil(28.6) = 29 zraka
    expected_beams = math.ceil(clearance / (2.0 * angle_increment))

    extended = detect_and_extend_disparities(
        ranges,
        angle_increment=angle_increment,
        car_width=car_width,
        safety_margin=safety_margin,
        disparity_threshold=0.5
    )

    # Zrake od 20 do 20 + expected_beams moraju biti proširene na 2.0m
    assert extended[20] == pytest.approx(2.0)
    assert extended[20 + expected_beams - 1] == pytest.approx(2.0)
    # Zrake nakon tog raspona moraju ostati netaknute (8.0m)
    if 20 + expected_beams < len(ranges):
        assert extended[20 + expected_beams] == pytest.approx(8.0)


def test_disparity_extension_backward():
    """
    Testira skok s dalekog prostora na bliski zid (npr. zrake 0..29 su 8m, a 30..49 su 2m).
    Zrake neposredno PRIJE indeksa 30 moraju biti skraćene na 2m.
    """
    ranges = np.ones(50) * 8.0
    ranges[30:] = 2.0
    angle_increment = math.radians(0.25)

    extended = detect_and_extend_disparities(
        ranges,
        angle_increment=angle_increment,
        car_width=0.30,
        safety_margin=0.10,
        disparity_threshold=0.5
    )

    # Zraka 29 (tik prije prepreke) mora biti spuštena na 2.0m
    assert extended[29] == pytest.approx(2.0)


def test_find_furthest_direction():
    """
    Provjerava da odabir cilja cilja u sredinu najšireg dubokog prolaza.
    """
    angles = np.linspace(-1.0, 1.0, 100)
    ranges = np.ones(100) * 2.0

    # Stvori dva prolaza: uski prolaz na indeksima 10..15 i široki prolaz na 60..90
    ranges[10:16] = 10.0
    ranges[60:91] = 10.0

    best_idx, best_angle, best_dist = find_furthest_direction(ranges, angles, far_ratio=0.9)

    # Sredina širokog prolaza je (60 + 90) // 2 = 75
    assert best_idx == 75
    assert best_dist == pytest.approx(10.0)
    assert best_angle == pytest.approx(angles[75])


def test_calculate_speed():
    # Ravno (kut 0.0) i slobodan prostor naprijed (10m) -> max_speed
    v_fast = calculate_speed(
        steering_angle=0.0,
        forward_distance=10.0,
        min_speed=1.5,
        max_speed=4.5
    )
    assert v_fast == pytest.approx(4.5)

    # Oštar zavoj (kut 0.34) -> min_speed
    v_slow = calculate_speed(
        steering_angle=0.34,
        forward_distance=10.0,
        min_speed=1.5,
        max_speed=4.5
    )
    assert v_slow == pytest.approx(1.5)

    # Prepreka blizu ispred (0.2m) -> minimalna brzina
    v_obstacle = calculate_speed(
        steering_angle=0.0,
        forward_distance=0.2,
        min_speed=1.5,
        max_speed=4.5,
        slow_down_distance=1.5
    )
    assert v_obstacle == pytest.approx(1.5)
