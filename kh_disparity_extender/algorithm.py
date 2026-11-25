"""
Algoritamska jezgra Disparity Extender metode (NumPy implementacija).

Ovaj modul sadrži čiste matematičke funkcije bez ROS 2 ovisnosti:
1. preprocess_scan: filtriranje vidnog polja, inf/nan vrijednosti i šuma
2. detect_and_extend_disparities: detekcija skokova i proširenje sjene prepreke za širinu auta
3. find_furthest_direction: odabir najdaljeg slobodnog smjera (središte najdublje zone)
4. calculate_speed: dinamičko prilagođavanje brzine ovisno o kutu skretanja i prostoru ispred
"""
import math
import numpy as np


def median_filter_1d(arr, size=3):
    """
    1D medijan filtar za uklanjanje pojedinačnih šumnih zraka senzora.
    Sprječava da jedna pogrešna zraka stvori lažni disparity.
    """
    if size <= 1 or len(arr) < size:
        return arr
    pad = size // 2
    padded = np.pad(arr, pad, mode='edge')
    windows = np.lib.stride_tricks.sliding_window_view(padded, size)
    return np.median(windows, axis=1)


def preprocess_scan(raw_ranges, raw_angles, fov_deg=150.0, max_range=12.0, filter_window_size=3):
    """
    Priprema sirovih LiDAR podataka:
    - Zadržava samo prednje zrake unutar zadanog vidnog polja (FOV)
    - Zamjenjuje nan i inf vrijednosti s max_range
    - Ograničava sve zrake na max_range
    - Primjenjuje medijan filtar za suzbijanje šuma
    """
    # 1. Kutno ograničenje (simetrično lijevo i desno od nule)
    half_fov = math.radians(fov_deg / 2.0)
    front_mask = np.abs(raw_angles) <= half_fov

    ranges = np.array(raw_ranges)[front_mask]
    angles = np.array(raw_angles)[front_mask]

    # 2. Rješavanje nepostojećih očitanja (nan, inf -> max_range)
    ranges = np.nan_to_num(ranges, nan=max_range, posinf=max_range, neginf=0.0)

    # 3. Ograničavanje na maksimalni domet
    ranges = np.minimum(ranges, max_range)

    # 4. Zaglađivanje šuma
    if filter_window_size > 1:
        ranges = median_filter_1d(ranges, size=filter_window_size)

    return ranges, angles


def detect_and_extend_disparities(ranges, angle_increment, car_width=0.31, safety_margin=0.15, disparity_threshold=0.35):
    """
    Detektira skokove (disparityje) između susjednih zraka i proširuje bližu prepreku.

    Matematički princip:
    - Skok se javlja na rubu prepreke gdje jedna zraka pogađa prepreku, a susjedna ide dalje.
    - Bliža zraka definira udaljenost prepreke: r_close = min(r[i], r[i+1])
    - Širina koju moramo maskirati je polovica auta plus sigurnosna margina:
        clearance = (car_width / 2.0) + safety_margin
    - Broj zraka potrebnih za pokrivanje te širine na udaljenosti r_close:
        N_beams = ceil(clearance / (r_close * angle_increment))
    - Zrake na DALJOJ strani skoka prepisuju se vrijednošću r_close.
    """
    extended = ranges.copy()
    n = len(ranges)
    clearance = (car_width / 2.0) + safety_margin

    for i in range(n - 1):
        diff = ranges[i + 1] - ranges[i]

        # Provjera prelazi li razlika prag disparityja
        if abs(diff) <= disparity_threshold:
            continue

        # Udaljenost bliže strane (prepreke)
        r_close = max(min(ranges[i], ranges[i + 1]), 0.05)

        # Računamo koliko zraka pokriva clearance na toj udaljenosti
        num_beams = math.ceil(clearance / (r_close * angle_increment))

        if diff > 0:
            # Skok s bližeg (i) na dalji (i+1) -> prepreka je s lijeve/donje strane
            # Maskiramo zrake na daljoj strani prema naprijed: od i+1 do i+1+num_beams
            start_idx = i + 1
            end_idx = min(i + 1 + num_beams, n)
            extended[start_idx:end_idx] = np.minimum(extended[start_idx:end_idx], r_close)
        else:
            # Skok s daljeg (i) na bliži (i+1) -> prepreka je s desne/gornje strane
            # Maskiramo zrake na daljoj strani prema natrag: od i - num_beams + 1 do i+1
            start_idx = max(0, i - num_beams + 1)
            end_idx = i + 1
            extended[start_idx:end_idx] = np.minimum(extended[start_idx:end_idx], r_close)

    return extended


def find_furthest_direction(ranges, angles, far_ratio=0.90):
    """
    Odabire najbolji ciljni smjer kretanja na proširenom LiDAR scanu.

    Strategija (Poboljšanje u odnosu na naivni argmax):
    - Pronalazi maksimalnu udaljenost u polju zraka: max_depth = max(ranges)
    - Pronalazi sve zrake koje dosežu barem far_ratio (npr. 90%) te dubine
    - Dijeli te "duboke" zrake u neprekinute grupe (prozore slobodnog prostora)
    - Odabire najširu grupu (najsigurniji i najširi prolaz)
    - Cilja u točno SREDIŠTE te najšire grupe
    """
    max_depth = np.max(ranges)

    # Indeksi zraka koje su dovoljno duboke
    deep_indices = np.where(ranges >= far_ratio * max_depth)[0]

    if len(deep_indices) == 0:
        # Sigurnosni fallback: ako ništa ne zadovoljava, uzmi apsolutni maksimum
        best_idx = int(np.argmax(ranges))
        return best_idx, float(angles[best_idx]), float(ranges[best_idx])

    # Grupiranje uzastopnih indeksa u kontinuirane raspone
    # Skok u indeksima veći od 1 znači prekid između dvije grupe
    breaks = np.where(np.diff(deep_indices) != 1)[0] + 1
    groups = np.split(deep_indices, breaks)

    # Odabiremo grupu s najvećim brojem zraka (najširi prolaz)
    widest_group = max(groups, key=len)

    # Ciljamo točno u sredinu tog prolaza
    best_idx = int(widest_group[len(widest_group) // 2])

    return best_idx, float(angles[best_idx]), float(ranges[best_idx])


def calculate_speed(steering_angle, forward_distance, min_speed=1.5, max_speed=4.5,
                    max_steer=0.34, steer_threshold=0.12, slow_down_distance=1.5):
    """
    Određuje sigurnu i optimalnu brzinu vozila ovisno o:
    1. Kutu skretanja: na ravnom vozi punom brzinom, u zavoju glatko usporava
    2. Udaljenosti ispred: ako je zid/prepreka direktno ispred unutar slow_down_distance, usporava na minimum
    """
    abs_steer = abs(steering_angle)

    # 1. Prilagodba s obzirom na kut skretanja
    if abs_steer <= steer_threshold:
        speed = max_speed
    else:
        # Linearno skaliranje između steer_threshold i max_steer
        steer_factor = (abs_steer - steer_threshold) / max(max_steer - steer_threshold, 1e-4)
        steer_factor = np.clip(steer_factor, 0.0, 1.0)
        speed = max_speed - steer_factor * (max_speed - min_speed)

    # 2. Sigurnosno kočenje ako je prepreka blizu ispred vozila
    if forward_distance < slow_down_distance:
        # Ako je prepreka unutar 0.5m, odmah spusti na minimalnu brzinu
        if forward_distance <= 0.5:
            speed = min_speed
        else:
            dist_factor = (forward_distance - 0.5) / (slow_down_distance - 0.5)
            dist_factor = float(np.clip(dist_factor, 0.0, 1.0))
            speed = min_speed + dist_factor * (speed - min_speed)

    # Osiguravamo da brzina nikad ne izađe iz zadanih granica
    speed = float(np.clip(speed, min_speed, max_speed))
    return speed
