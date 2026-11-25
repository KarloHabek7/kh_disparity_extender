"""
Vizualizacijske funkcije za RViz (teme /scan_proc, /scan_extended, /target_marker).
Ovaj modul služi isključivo za grafički prikaz u RViz-u i ne utječe na rad samog algoritma.
"""
import math

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

# Trajanje prikaza markera u RViz-u (pola sekunde)
LIFETIME = Duration(sec=0, nanosec=500_000_000)


def _point(distance, angle):
    """Pretvara polarne koordinate (udaljenost, kut) senzora u točku (x, y)."""
    return Point(
        x=float(distance * math.cos(angle)),
        y=float(distance * math.sin(angle)),
        z=0.0
    )


def scan_msg(original_scan, ranges, angles):
    """Kreira novu LaserScan poruku s modificiranim zrakama (za RViz)."""
    msg = LaserScan()
    msg.header = original_scan.header
    msg.angle_min = float(angles[0])
    msg.angle_max = float(angles[-1])
    msg.angle_increment = original_scan.angle_increment
    msg.range_min = original_scan.range_min
    msg.range_max = original_scan.range_max
    msg.ranges = [float(r) for r in ranges]
    return msg


def target_marker(scan, distance, angle):
    """
    Kreira zelenu strelicu u RViz-u od centra vozila prema odabranom ciljnom smjeru.
    """
    marker = Marker()
    marker.header = scan.header
    marker.ns = 'disparity_target'
    marker.id = 0
    marker.type = Marker.ARROW
    marker.action = Marker.ADD

    # Strelica počinje u ishodištu (vozilo) i završava na ciljnoj točki
    p_start = Point(x=0.0, y=0.0, z=0.0)
    p_end = _point(distance, angle)
    marker.points = [p_start, p_end]

    # Dimenzije strelice (debljina stabla, glave)
    marker.scale.x = 0.08  # debljina tijela
    marker.scale.y = 0.16  # širina vrha
    marker.scale.z = 0.20  # visina vrha

    # Jarko zelena boja
    marker.color = ColorRGBA(r=0.0, g=1.0, b=0.2, a=1.0)
    marker.lifetime = LIFETIME
    return marker
