#!/usr/bin/env python3
"""
Class-based ros2-rosbag GPS reader, UTM converter, outlier filter and plotter.
"""
import argparse
import numpy as np
import os
import sys
from typing import List, Tuple, Optional

try:
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except Exception as e:
    sys.stderr.write("Missing ROS2 Python packages. Source ROS2 and ensure rosbag2_py, rclpy and rosidl_runtime_py are installed.\n")
    raise

try:
    from pyproj import Transformer
    import matplotlib.pyplot as plt
    import matplotlib as mpl
    import numpy as np
except Exception as e:
    sys.stderr.write("Missing plotting/coord packages. Install pyproj, matplotlib and numpy.\n")
    raise


class RosbagReader:
    """Open rosbag2 (sqlite3 or mcap) and read messages."""

    def __init__(self, uri: str):
        self.uri = uri
        self.storage_id = ""
        self.reader = SequentialReader()

    def detect_storage_id(self) -> str:
        if os.path.isfile(self.uri):
            lower = self.uri.lower()
            if lower.endswith(".mcap") or lower.endswith(".mcap.zst"):
                return "mcap"
            if lower.endswith(".db3") or lower.endswith(".sqlite3"):
                return "sqlite3"
        if os.path.isdir(self.uri):
            meta = os.path.join(self.uri, "metadata.yaml")
            if os.path.exists(meta):
                try:
                    with open(meta, "r") as f:
                        for line in f:
                            if "storage_identifier" in line:
                                parts = line.split(":", 1)
                                if len(parts) > 1:
                                    return parts[1].strip().strip(' "\'')
                except Exception:
                    pass
            for fname in os.listdir(self.uri):
                lf = fname.lower()
                if lf.endswith(".mcap") or lf.endswith(".mcap.zst"):
                    return "mcap"
                if lf.endswith(".db3") or lf.endswith(".sqlite3"):
                    return "sqlite3"
        return ""  # let rosbag2 try autodetect

    def open(self) -> str:
        self.storage_id = self.detect_storage_id()
        opts = StorageOptions(uri=self.uri, storage_id=self.storage_id)
        conv = ConverterOptions("", "")
        self.reader.open(opts, conv)
        return self.storage_id

    def list_topics(self):
        return self.reader.get_all_topics_and_types()

    def get_topic_type(self, topic_name: str) -> Optional[str]:
        topics = self.list_topics()
        for t in topics:
            if t.name == topic_name:
                return t.type
        return None

    def read_topic_messages(self, topic_name: str):
        # create a fresh reader to start from beginning
        r = SequentialReader()
        storage_options = StorageOptions(uri=self.uri, storage_id=self.storage_id)
        conv = ConverterOptions("", "")
        r.open(storage_options, conv)
        while r.has_next():
            try:
                topic, data, timestamp = r.read_next()
            except Exception:
                break
            if topic != topic_name:
                continue
            yield topic, data, timestamp


class NavSatFixProcessor:
    """Convert NavSatFix messages into dicts and transform lat/lon lists to UTM."""

    @staticmethod
    def deserialize_message(raw_serialized, msg_type: str):
        if hasattr(raw_serialized, "serialized_data"):
            raw = raw_serialized.serialized_data
        else:
            raw = raw_serialized
        MsgClass = get_message(msg_type)
        return deserialize_message(raw, MsgClass)

    @staticmethod
    def msg_to_record(msg, timestamp_ns) -> dict:
        header = getattr(msg, "header", None)
        stamp = getattr(header, "stamp", None)
        if stamp is not None and hasattr(stamp, "sec"):
            t = float(getattr(stamp, "sec", 0)) + float(getattr(stamp, "nanosec", 0)) * 1e-9
        else:
            t = float(timestamp_ns) / 1e9 if timestamp_ns is not None else None
        return {
            "time": t,
            "latitude": getattr(msg, "latitude", None),
            "longitude": getattr(msg, "longitude", None),
            "altitude": getattr(msg, "altitude", None),
        }

    @staticmethod
    def latlon_lists_to_utm(lat_list: List[float], lon_list: List[float]) -> Tuple[List[float], List[float], Optional[Tuple[int, int]]]:
        if not lat_list:
            return [], [], None
        lat0, lon0 = lat_list[0], lon_list[0]
        zone = int((lon0 + 180) / 6) + 1
        epsg = 32600 + zone if lat0 >= 0 else 32700 + zone
        transformer = Transformer.from_crs("EPSG:4326", f"EPSG:{epsg}", always_xy=True)
        xs, ys = transformer.transform(lon_list, lat_list)
        return list(xs), list(ys), (zone, epsg)


class SpeedOutlierFilter:
    """
    Remove outliers from UTM tracks based on a maximum allowed speed.

    The filter checks the distance between points relative to the time difference and a
    maximum speed (m/s). By default it uses walking speed ~1.5 m/s.

    Parameters:
    - max_speed: maximum allowed speed in m/s (default 1.5)
    - max_factor: safety multiplier for allowed distance (default 1.5)
    - mode: 'consecutive' (compare each point to previous sample) or
            'last_accepted' (compare to last kept point) — default 'last_accepted'
    - eps_dt: small fallback delta time (seconds) used when timestamps are None or equal
    """
    def __init__(self, max_speed: float = 1.5, max_factor: float = 1.5,
                 mode: str = "last_accepted", eps_dt: float = 1e-3):
        self.max_speed = float(max_speed)
        self.max_factor = float(max_factor)
        if mode not in ("consecutive", "last_accepted"):
            raise ValueError("mode must be 'consecutive' or 'last_accepted'")
        self.mode = mode
        self.eps_dt = float(eps_dt)

    @staticmethod
    def _dist(x1: float, y1: float, x2: float, y2: float) -> float:
        dx = x2 - x1
        dy = y2 - y1
        return (dx * dx + dy * dy) ** 0.5

    def filter(self, xs: List[float], ys: List[float], times: List[Optional[float]]):
        """
        Filter UTM points.

        Inputs:- xs, ys: lists of UTM eastings/northings (meters) times: list of timestamps in seconds (floats) or None (same length)
        Returns:- xs_f, ys_f, times_f: filtered lists (ordered)- removed_indices: indices (into original lists) that were removed as outliers
        """
        if not xs or not ys:
            return [], [], [], []

        n = len(xs)
        if not (len(ys) == n == len(times)):
            raise AssertionError("xs, ys and times must have same length")

        kept_idx = [0]
        removed = []

        for i in range(1, n):
            # choose baseline index for distance/time comparison
            if self.mode == "last_accepted":
                j = kept_idx[-1]
            else:  # consecutive
                j = i - 1

            dist = self._dist(xs[j], ys[j], xs[i], ys[i])
            t_i = times[i]
            t_j = times[j]
            if t_i is None or t_j is None:
                dt = self.eps_dt
            else:
                dt = max(t_i - t_j, self.eps_dt)

            allowed = self.max_speed * dt * self.max_factor

            if dist > allowed:
                # point i is an outlier
                removed.append(i)
                # do not append to kept_idx
            else:
                kept_idx.append(i)

        xs_f = [xs[i] for i in kept_idx]
        ys_f = [ys[i] for i in kept_idx]
        times_f = [times[i] for i in kept_idx]
        return xs_f, ys_f, times_f, removed


class RouteSimplifier:
    """
    Simplify a lat/lon/alt track into route waypoints.

    Provides:
      - simplify_by_max_waypoints(max_waypoints, angle_weight=0.0)
      - simplify_by_distance_tolerance(dist_tol_m)
      - simplify_by_distance_and_bearing(dist_tol_m, angle_weight)
    """

    def __init__(self, points):
        # Normalize input to lists of lat, lon, alt
        lats, lons, alts = [], [], []
        if not points:
            self.lats = self.lons = self.alts = []
        else:
            first = points[0]
            if isinstance(first, dict):
                for p in points:
                    lats.append(float(p.get("latitude") or p.get("lat")))
                    lons.append(float(p.get("longitude") or p.get("lon") or p.get("long")))
                    alts.append(float(p.get("altitude", 0)))
            else:
                for p in points:
                    lats.append(float(p[0]))
                    lons.append(float(p[1]))
                    alts.append(float(p[2]) if len(p) > 2 else 0.0)
            self.lats = lats
            self.lons = lons
            self.alts = alts

        # local UTM transformer based on first point
        if self.lats and self.lons:
            zone = int((self.lons[0] + 180) / 6) + 1
            epsg = 32600 + zone if self.lats[0] >= 0 else 32700 + zone
            self._transformer = Transformer.from_crs("EPSG:4326", f"EPSG:{epsg}", always_xy=True)
        else:
            self._transformer = None

        self._xs, self._ys = self._project_to_xy()

    def _project_to_xy(self):
        if not self._transformer or not self.lats:
            return [], []
        xs, ys = self._transformer.transform(self.lons, self.lats)
        return list(xs), list(ys)

    @staticmethod
    def _perp_distance(px, py, x1, y1, x2, y2):
        dx = x2 - x1
        dy = y2 - y1
        if dx == 0 and dy == 0:
            return ((px - x1) ** 2 + (py - y1) ** 2) ** 0.5
        t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)
        if t < 0:
            return ((px - x1) ** 2 + (py - y1) ** 2) ** 0.5
        if t > 1:
            return ((px - x2) ** 2 + (py - y2) ** 2) ** 0.5
        projx = x1 + t * dx
        projy = y1 + t * dy
        return ((px - projx) ** 2 + (py - projy) ** 2) ** 0.5

    @staticmethod
    def _bearing_xy(x1, y1, x2, y2):
        import math
        return math.atan2(y2 - y1, x2 - x1)

    def _rdp_indices(self, eps, angle_weight=0.0):
        n = len(self._xs)
        if n == 0:
            return []
        stack = [(0, n - 1)]
        keep = [False] * n
        keep[0] = keep[-1] = True

        import math
        while stack:
            i, j = stack.pop()
            x1, y1 = self._xs[i], self._ys[i]
            x2, y2 = self._xs[j], self._ys[j]
            max_err = -1.0
            idx = None
            base_bearing = self._bearing_xy(x1, y1, x2, y2)
            seg_len = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
            for k in range(i + 1, j):
                px, py = self._xs[k], self._ys[k]
                dist = self._perp_distance(px, py, x1, y1, x2, y2)
                if angle_weight and seg_len > 0:
                    b = self._bearing_xy(x1, y1, px, py)
                    angle_diff = abs((b - base_bearing + math.pi) % (2 * math.pi) - math.pi)
                    err = dist + angle_weight * angle_diff * seg_len
                else:
                    err = dist
                if err > max_err:
                    max_err = err
                    idx = k
            if max_err is not None and max_err > eps and idx is not None:
                keep[idx] = True
                stack.append((i, idx))
                stack.append((idx, j))
        return [i for i, v in enumerate(keep) if v]

    def simplify_by_max_waypoints(self, max_waypoints: int, angle_weight=0.0, max_iters=40):
        """
        Reduce the track to at most `max_waypoints` using the Ramer–Douglas–Peucker (RDP)
        algorithm. We binary-search an RDP epsilon (perpendicular-error threshold) until
        the RDP result has <= max_waypoints points.

        - angle_weight is forwarded to the RDP error metric (0 => classic RDP).
        - If binary search cannot reach the exact count, we fall back to a simple
          uniform downsampling of the best candidate.
        """
        n = len(self._xs)
        if n == 0:
            return []
        if max_waypoints <= 2:
            return self._indices_to_waypoints([0, n - 1])

        # Upper bound for epsilon: maximum perpendicular distance of any point to the
        # straight line between start and end (times a small factor).
        x1, y1 = self._xs[0], self._ys[0]
        x2, y2 = self._xs[-1], self._ys[-1]
        maxd = 0.0
        for k in range(1, n - 1):
            d = self._perp_distance(self._xs[k], self._ys[k], x1, y1, x2, y2)
            if d > maxd:
                maxd = d

        lo, hi = 0.0, maxd * 2.0 + 1.0
        best_idx = list(range(n))

        # Binary search epsilon to find RDP result with <= max_waypoints points.
        for _ in range(max_iters):
            mid = (lo + hi) / 2.0
            # _rdp_indices implements the Ramer–Douglas–Peucker selection
            idx = self._rdp_indices(mid, angle_weight=angle_weight)
            if len(idx) <= max_waypoints:
                best_idx = idx
                hi = mid
            else:
                lo = mid
            if hi - lo < 1e-3:
                break

        # If still too many points, downsample the best candidate uniformly to meet the limit.
        if len(best_idx) > max_waypoints:
            step = max(1, int(len(best_idx) / max_waypoints))
            best_idx = [best_idx[i] for i in range(0, len(best_idx), step)]
            if best_idx[-1] != n - 1:
                best_idx.append(n - 1)

        return self._indices_to_waypoints(best_idx)

    def simplify_by_distance_tolerance(self, dist_tol_m: float):
        """
        Simplify track using classic RDP so maximum perpendicular deviation <= dist_tol_m (meters).
        """
        idx = self._rdp_indices(dist_tol_m, angle_weight=0.0)
        return self._indices_to_waypoints(idx)

    def simplify_by_distance_and_bearing(self, dist_tol_m: float, angle_weight: float = 0.1):
        """
        Simplify using combined distance + bearing error:
          error = perpendicular_distance + angle_weight * angle_diff * segment_length
        where angle_diff is in radians. Larger angle_weight makes bearing deviations more costly.
        """
        idx = self._rdp_indices(dist_tol_m, angle_weight=angle_weight)
        return self._indices_to_waypoints(idx)

    def _indices_to_waypoints(self, indices):
        """
        Convert list of indices (into the original track) to waypoint dicts
        [{'latitude':..., 'longitude':..., 'altitude':...}, ...].
        """
        if not indices:
            return []
        indices = sorted(indices)
        waypoints = []
        for i in indices:
            lat = float(self.lats[i])
            lon = float(self.lons[i])
            alt = float(self.alts[i]) if i < len(self.alts) else 0.0
            waypoints.append({
                "latitude": lat,
                "longitude": lon,
                "altitude": alt,
            })
        return waypoints


class FixedWingPathSmoother:
    """
    Smooth a route (UTM coordinates) for fixed-wing flight by inserting interpolation
    waypoints using a cubic Hermite spline (piecewise Hermite interpolation).

    - tension: scales tangents (0 => straight lines between points, 1 => full central-difference tangents)
    - points_per_segment: number of interpolated samples per input segment
    """
    def __init__(self, tension: float = 0.5, points_per_segment: int = 20):
        self.tension = float(tension)
        self.points_per_segment = int(points_per_segment)

    @staticmethod
    def _compute_tangents(xs, ys, tension):
        n = len(xs)
        tangents = []
        if n == 1:
            return [(0.0, 0.0)]
        for i in range(n):
            if i == 0:
                dx = xs[1] - xs[0]
                dy = ys[1] - ys[0]
            elif i == n - 1:
                dx = xs[-1] - xs[-2]
                dy = ys[-1] - ys[-2]
            else:
                dx = 0.5 * (xs[i + 1] - xs[i - 1])
                dy = 0.5 * (ys[i + 1] - ys[i - 1])
            tangents.append((tension * dx, tension * dy))
        return tangents

    @staticmethod
    def _hermite_segment(p0x, p0y, p1x, p1y, m0x, m0y, m1x, m1y, num):
        import numpy as np
        if num <= 0:
            return [], []
        t = np.linspace(0.0, 1.0, num, endpoint=False)
        t2 = t * t
        t3 = t2 * t
        h00 = 2.0 * t3 - 3.0 * t2 + 1.0
        h10 = t3 - 2.0 * t2 + t
        h01 = -2.0 * t3 + 3.0 * t2
        h11 = t3 - t2
        xs = h00 * p0x + h10 * m0x + h01 * p1x + h11 * m1x
        ys = h00 * p0y + h10 * m0y + h01 * p1y + h11 * m1y
        return xs, ys

    def smooth(self, xs: List[float], ys: List[float], points_per_segment: Optional[int] = None):
        """
        Return (sx, sy) with interpolated points. If points_per_segment is None theobject's default is used.
        """
        if not xs or not ys:
            return [], []
        if len(xs) != len(ys):
            raise AssertionError("xs and ys must have same length")
        n = len(xs)
        if n < 2:
            return list(xs), list(ys)

        pps = points_per_segment if points_per_segment is not None else self.points_per_segment
        tangents = self._compute_tangents(xs, ys, self.tension)

        sx = []
        sy = []
        for i in range(n - 1):
            p0x, p0y = xs[i], ys[i]
            p1x, p1y = xs[i + 1], ys[i + 1]
            m0x, m0y = tangents[i]
            m1x, m1y = tangents[i + 1]
            seg_x, seg_y = self._hermite_segment(p0x, p0y, p1x, p1y, m0x, m0y, m1x, m1y, max(1, int(pps)))
            # seg arrays exclude the endpoint; append them
            if isinstance(seg_x, (list, tuple)):
                sx.extend(seg_x)
                sy.extend(seg_y)
            else:
                sx.extend(seg_x.tolist())
                sy.extend(seg_y.tolist())
        # append final original point
        sx.append(xs[-1])
        sy.append(ys[-1])
        return list(sx), list(sy)


class UTMPlotter:
    """Plot raw, filtered and three route simplifications plus a smoothed fixed-wing route (6 subplots)."""

    @staticmethod
    def plot_all(orig_xs: List[float], orig_ys: List[float], orig_times: List[Optional[float]],
                 filt_xs: List[float], filt_ys: List[float], filt_times: List[Optional[float]],
                 removed_idx: List[int], zone_info: Optional[Tuple[int, int]],
                 out: str, show: bool,
                 route_max: Optional[Tuple[List[float], List[float]]] = None,
                 route_dist: Optional[Tuple[List[float], List[float]]] = None,
                 route_dist_bearing: Optional[Tuple[List[float], List[float]]] = None,
                 route_smooth: Optional[Tuple[List[float], List[float]]] = None):
        import matplotlib.pyplot as plt
        import matplotlib as mpl
        import numpy as np

        # create 2 rows x 3 cols (6 subplots)
        fig, axs = plt.subplots(2, 3, figsize=(20, 10))
        ax_raw = axs[0, 0]
        ax_filt = axs[0, 1]
        ax_max = axs[0, 2]
        ax_dist = axs[1, 0]
        ax_db = axs[1, 1]
        ax_smooth = axs[1, 2]

        def mark_start_end(ax, xs, ys):
            if xs and ys:
                ax.scatter(xs[0], ys[0], c='green', s=60, marker='*', edgecolor='k', label='start')
                ax.scatter(xs[-1], ys[-1], c='red', s=60, marker='X', edgecolor='k', label='end')

        # 1) Raw points (black)
        if orig_xs and orig_ys:
            ax_raw.plot(orig_xs, orig_ys, '-', color='black', linewidth=0.8, label='raw track')
            ax_raw.scatter(orig_xs, orig_ys, c='black', s=18, edgecolor='none', alpha=0.9)
            mark_start_end(ax_raw, orig_xs, orig_ys)
        ax_raw.set_title('1) Raw points')
        ax_raw.set_xlabel('Easting (m)'); ax_raw.set_ylabel('Northing (m)')
        ax_raw.axis('equal'); ax_raw.grid(True); ax_raw.legend(loc='best')

        # 2) Filtered points (colored by time if available)
        times_arr = np.array([t if t is not None else np.nan for t in (filt_times or orig_times)], dtype=float)
        if filt_xs and filt_ys:
            if np.all(np.isnan(times_arr)):
                ax_filt.plot(filt_xs, filt_ys, '-o', color='C0', markersize=4, label='filtered')
            else:
                valid_mask = np.isfinite(times_arr)
                if np.any(valid_mask):
                    tmin = times_arr[valid_mask].min(); tmax = times_arr[valid_mask].max()
                else:
                    tmin, tmax = 0.0, 1.0
                cmap = plt.get_cmap('viridis')
                norm = mpl.colors.Normalize(vmin=tmin, vmax=tmax)
                sc = ax_filt.scatter(filt_xs, filt_ys, c=times_arr, cmap=cmap, norm=norm, s=36, edgecolor='k', linewidth=0.25)
                cbar = fig.colorbar(mpl.cm.ScalarMappable(cmap=cmap, norm=norm), ax=ax_filt, fraction=0.046, pad=0.04)
                cbar.set_label('time (s)')
            mark_start_end(ax_filt, filt_xs, filt_ys)
            # show removed only on raw/filtered
            if removed_idx and orig_xs and orig_ys:
                rx = [orig_xs[i] for i in removed_idx if 0 <= i < len(orig_xs)]
                ry = [orig_ys[i] for i in removed_idx if 0 <= i < len(orig_ys)]
                if rx and ry:
                    ax_filt.scatter(rx, ry, c='red', marker='x', s=70, linewidths=2, label='removed')
        ax_filt.set_title('2) Filtered points')
        ax_filt.set_xlabel('Easting (m)'); ax_filt.set_ylabel('Northing (m)')
        ax_filt.axis('equal'); ax_filt.grid(True); ax_filt.legend(loc='best')

        # 3) Max waypoints
        if orig_xs and orig_ys:
            ax_max.plot(orig_xs, orig_ys, '-', color="#000000", linewidth=1)
            ax_max.scatter(orig_xs, orig_ys, c='#f6f6f6', s=12, edgecolor='none', alpha=0.6)
        if route_max and route_max[0] and route_max[1]:
            rx, ry = route_max
            ax_max.plot(rx, ry, '-D', color='magenta', linewidth=2, markersize=6, label=f'max_waypoints (n={len(rx)})')
            ax_max.scatter(rx, ry, c='magenta', marker='D', s=70, edgecolor='k')
        mark_start_end(ax_max, filt_xs, filt_ys)
        ax_max.set_title('3) Max waypoints (RDP)')
        ax_max.set_xlabel('Easting (m)'); ax_max.set_ylabel('Northing (m)')
        ax_max.axis('equal'); ax_max.grid(True); ax_max.legend(loc='best')

        # 4) Distance tolerance
        if orig_xs and orig_ys:
            ax_dist.plot(orig_xs, orig_ys, '-', color="#000000", linewidth=1)
            ax_dist.scatter(orig_xs, orig_ys, c='#f6f6f6', s=12, edgecolor='none', alpha=0.6)
        if route_dist and route_dist[0] and route_dist[1]:
            rx, ry = route_dist
            ax_dist.plot(rx, ry, '-o', color='orange', linewidth=2, markersize=6, label=f'dist_tol (n={len(rx)})')
            ax_dist.scatter(rx, ry, c='orange', marker='o', s=70, edgecolor='k')
        mark_start_end(ax_dist, filt_xs, filt_ys)
        ax_dist.set_title('4) Distance tolerance (RDP)')
        ax_dist.set_xlabel('Easting (m)'); ax_dist.set_ylabel('Northing (m)')
        ax_dist.axis('equal'); ax_dist.grid(True); ax_dist.legend(loc='best')

        # 5) Distance + Bearing
        if orig_xs and orig_ys:
            ax_db.plot(orig_xs, orig_ys, '-', color="#040404", linewidth=1)
            ax_db.scatter(orig_xs, orig_ys, c='#f6f6f6', s=12, edgecolor='none', alpha=0.6)
        if route_dist_bearing and route_dist_bearing[0] and route_dist_bearing[1]:
            rx, ry = route_dist_bearing
            ax_db.plot(rx, ry, '-s', color='purple', linewidth=2, markersize=6, label=f'dist_bearing (n={len(rx)})')
            ax_db.scatter(rx, ry, c='purple', marker='s', s=70, edgecolor='k')
        mark_start_end(ax_db, filt_xs, filt_ys)
        ax_db.set_title('5) Distance + Bearing (RDP variant)')
        ax_db.set_xlabel('Easting (m)'); ax_db.set_ylabel('Northing (m)')
        ax_db.axis('equal'); ax_db.grid(True); ax_db.legend(loc='best')

        # 6) Smoothed fixed-wing route (Hermite spline)
        if orig_xs and orig_ys:
            ax_smooth.plot(orig_xs, orig_ys, '-', color='#f0f0f0', linewidth=1)
            ax_smooth.scatter(orig_xs, orig_ys, c='#f6f6f6', s=12, edgecolor='none', alpha=0.6)
        if route_smooth and route_smooth[0] and route_smooth[1]:
            sx, sy = route_smooth
            ax_smooth.plot(sx, sy, '-', color='cyan', linewidth=2, label='smoothed fixed-wing path')
            ax_smooth.scatter(sx[0::max(1, int(len(sx)/20))], sy[0::max(1, int(len(sy)/20))], c='cyan', s=20, edgecolor='k')
        mark_start_end(ax_smooth, filt_xs, filt_ys)
        ax_smooth.set_title('6) Smoothed fixed-wing route (Hermite)')
        ax_smooth.set_xlabel('Easting (m)'); ax_smooth.set_ylabel('Northing (m)')
        ax_smooth.axis('equal'); ax_smooth.grid(True)
        if route_smooth:
            ax_smooth.legend(loc='best')

        if zone_info is not None:
            z, epsg = zone_info
            fig.suptitle(f"Trajectory UTM zone {z} (EPSG:{epsg})", fontsize=12)

        fig.text(0.5, 0.01,
                 "Waypoints computed using the Ramer–Douglas–Peucker (RDP) algorithm. "
                 "Smoothed path uses a cubic Hermite spline (tension controls tangents).",
                 ha='center', va='bottom', fontsize=9)

        plt.tight_layout(rect=[0, 0.03, 1, 0.96])
        plt.savefig(out, dpi=200)
        print(f"Wrote plot to {out} (raw:{len(orig_xs)} filt:{len(filt_xs)} max:{len(route_max[0]) if route_max else 0} dist:{len(route_dist[0]) if route_dist else 0} db:{len(route_dist_bearing[0]) if route_dist_bearing else 0} smooth:{len(route_smooth[0]) if route_smooth else 0} removed:{len(removed_idx)})")
        if show:
            plt.show()


class QGCWaypointsExporter:
    """
    Convert UTM or lat/lon waypoints to geographic coordinates and write a QGroundControl
    compatible waypoint file (WPL 110).

    Usage:
      - If you have UTM points (xs, ys) provide zone_info=(zone, epsg) from NavSatFixProcessor.latlon_lists_to_utm.
        call to_geographic_from_utm(xs, ys, zone_info) to get [(lat, lon), ...]
      - If you already have lat/lon/alt waypoints (list of dicts as produced by RouteSimplifier) call
        write_wpl_file(waypoints, filename)

    The WPL format written is the legacy "QGC WPL 110" format which QGroundControl accepts:
    Header: "QGC WPL 110"
    Each line: seq\tcurrent\tframe\tcommand\tparam1\tparam2\tparam3\tparam4\tlat\tlon\talt\tautocontinue
    - frame: 3 (MAV_FRAME_GLOBAL_RELATIVE_ALT)
    - command: 16 (MAV_CMD_NAV_WAYPOINT)
    - params are set to zeros (no hold/accept radius/yaw)
    - first waypoint is marked current (current=1), others 0
    """

    def __init__(self):
        pass

    @staticmethod
    def to_geographic_from_utm(xs: List[float], ys: List[float], zone_info: Optional[Tuple[int, int]]):
        """
        Convert UTM eastings/northings to geographic lat/lon using provided zone_info (zone, epsg).
        Returns list of (lat, lon) tuples.
        """
        if not xs or not ys or zone_info is None:
            return []
        _, epsg = zone_info
        # inverse transform from UTM (epsg) to EPSG:4326
        transformer = Transformer.from_crs(f"EPSG:{epsg}", "EPSG:4326", always_xy=True)
        lons, lats = transformer.transform(xs, ys)
        return list(zip(list(lats), list(lons)))

    @staticmethod
    def _normalize_waypoints_input(waypoints):
        """
        Accept either:
          - list of dicts {'latitude','longitude','altitude'}
          - list of tuples (lat, lon, alt)
        Return list of (lat, lon, alt)
        """
        out = []
        for p in waypoints or []:
            if isinstance(p, dict):
                lat = float(p.get("latitude") or p.get("lat"))
                lon = float(p.get("longitude") or p.get("lon"))
                alt = float(p.get("altitude", 0.0))
            else:
                lat = float(p[0]); lon = float(p[1]); alt = float(p[2]) if len(p) > 2 else 0.0
            out.append((lat, lon, alt))
        return out

    def write_wpl_file(self, waypoints, filename: str, hold_time: float = 0.0, autocontinue: int = 1) -> str:
        """
        Write waypoints to a QGroundControl WPL file.

        - waypoints: list of dicts or tuples (lat, lon, alt). If you have UTM points,
          convert first using to_geographic_from_utm and build tuples with alt.
        - filename: output file path
        - hold_time: param1 (seconds) for each MAV_CMD_NAV_WAYPOINT
        - autocontinue: typically 1

        Returns the filename written.
        """
        wps = self._normalize_waypoints_input(waypoints)
        if not wps:
            raise ValueError("No waypoints to write")

        # QGC WPL 110 header
        header = "QGC WPL 110\n"
        lines = [header]
        frame = 3      # MAV_FRAME_GLOBAL_RELATIVE_ALT
        command = 16   # MAV_CMD_NAV_WAYPOINT
        param1 = float(hold_time)
        param2 = 0.0   # acceptance radius
        param3 = 0.0   # pass radius / unused
        param4 = 0.0   # yaw

        for seq, (lat, lon, alt) in enumerate(wps):
            current = 1 if seq == 0 else 0
            # Build line: seq, current, frame, command, param1, param2, param3, param4, lat, lon, alt, autocontinue
            line = "{idx}\t{cur}\t{frame}\t{cmd}\t{p1}\t{p2}\t{p3}\t{p4}\t{lat:.9f}\t{lon:.9f}\t{alt:.3f}\t{ac}\n".format(
                idx=seq, cur=current, frame=frame, cmd=command,
                p1=param1, p2=param2, p3=param3, p4=param4,
                lat=lat, lon=lon, alt=alt, ac=autocontinue
            )
            lines.append(line)

        # write file atomically
        tmp = filename + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            f.writelines(lines)
        os.replace(tmp, filename)
        return filename

    def write_wpl_from_utm(self, xs: List[float], ys: List[float], alts: Optional[List[float]],
                           zone_info: Optional[Tuple[int, int]], filename: str,
                           hold_time: float = 0.0, autocontinue: int = 1) -> str:
        """
        Convert UTM points to geographic and write WPL file.
        - xs, ys: UTM coordinates
        - alts: list of altitudes (optional, will be filled with zeros if None or length mismatch)
        - zone_info: (zone, epsg) returned by NavSatFixProcessor.latlon_lists_to_utm
        """
        if not xs or not ys:
            raise ValueError("Empty UTM lists")
        latlon = self.to_geographic_from_utm(xs, ys, zone_info)
        alts = alts or [0.0] * len(latlon)
        if len(alts) < len(latlon):
            # pad
            alts = alts + [0.0] * (len(latlon) - len(alts))
        waypoints = [(lat, lon, alt) for (lat, lon), alt in zip(latlon, alts)]
        return self.write_wpl_file(waypoints, filename, hold_time=hold_time, autocontinue=autocontinue)


class GPSApp:
    """Command-line application orchestrating reading, processing, filtering and plotting."""

    def parse_args(self):
        p = argparse.ArgumentParser(description="Read /mavros/global_position/raw/fix from ros2 bag, convert to UTM, filter and plot.")
        p.add_argument("bag", help="Path to rosbag2 folder or .mcap/.db3 file")
        p.add_argument("--topic", "-t", default="/mavros/global_position/raw/fix", help="Topic to read")
        p.add_argument("--out", "-o", default="traj_utm.png", help="Output image filename")
        p.add_argument("--show", "-s", action="store_true", help="Show plot interactively")
        p.add_argument("--list", "-l", action="store_true", help="List topics and exit")
        p.add_argument("--max-speed", type=float, default=1.5, help="Max speed m/s for outlier filter (default walking speed 1.5 m/s)")
        p.add_argument("--max-factor", type=float, default=1.5, help="Safety factor for speed threshold")
        p.add_argument("--route-max-wp", type=int, default=20, help="Maximum route waypoints to generate (default 20)")
        p.add_argument("--route-method", choices=["max_waypoints", "dist_tol", "dist_bearing"],
                       default="max_waypoints", help="Route simplification method")
        p.add_argument("--route-dist-tol", type=float, default=5.0,
                       help="Distance tolerance in meters for RDP (used with dist_tol or dist_bearing)")
        p.add_argument("--route-angle-weight", type=float, default=0.1,
                       help="Angle weight (unitless) for dist_bearing method")
        self.args = p.parse_args()

    def run(self):
        self.parse_args()
        rdr = RosbagReader(self.args.bag)
        try:
            used = rdr.open()
            if used:
                print(f"Opened bag '{self.args.bag}' with storage_id='{used}'")
            else:
                print(f"Opened bag '{self.args.bag}' with auto-detected storage plugin")
        except Exception as e:
            print(f"Failed to open bag '{self.args.bag}': {e}")
            return

        topics = rdr.list_topics()
        if self.args.list:
            for t in topics:
                print(f"{t.name}    {t.type}")
            return

        msg_type = rdr.get_topic_type(self.args.topic)
        if msg_type is None:
            print(f"Topic {self.args.topic} not found in bag. Available topics:")
            for t in topics:
                print(f"  {t.name} ({t.type})")
            return

        # read messages and build records
        proc = NavSatFixProcessor()
        records = []
        for topic, data, timestamp in rdr.read_topic_messages(self.args.topic):
            try:
                msg = proc.deserialize_message(data, msg_type)
            except Exception as e:
                print(f"Deserialize error: {e}")
                continue
            rec = proc.msg_to_record(msg, timestamp)
            if rec["latitude"] is None or rec["longitude"] is None:
                continue
            records.append(rec)

        if not records:
            print("No valid NavSatFix messages found.")
            return

        lats = [r["latitude"] for r in records]
        lons = [r["longitude"] for r in records]
        times = [r["time"] for r in records]
        xs, ys, zone_info = proc.latlon_lists_to_utm(lats, lons)

        # filter outliers
        filt = SpeedOutlierFilter(max_speed=self.args.max_speed, max_factor=self.args.max_factor)
        xs_f, ys_f, times_f, removed = filt.filter(xs, ys, times)
        if removed:
            print(f"Removed {len(removed)} outlier points (indices): {removed}")

        # generate simplified route waypoints (compute all methods) using filtered track
        route_max_xy = route_dist_xy = route_dist_bearing_xy = None

        # build filtered lat/lon list (keep indices not removed)
        removed_set = set(removed)
        filtered_points = []
        for i, rec in enumerate(records):
            if i in removed_set:
                continue
            filtered_points.append((rec["latitude"], rec["longitude"], rec.get("altitude", 0.0)))

        if filtered_points:
            rs = RouteSimplifier(filtered_points)
            # max_waypoints
            if self.args.route_max_wp and self.args.route_max_wp > 0:
                wp_max = rs.simplify_by_max_waypoints(self.args.route_max_wp)
                if wp_max:
                    lat_max = [wp["latitude"] for wp in wp_max]
                    lon_max = [wp["longitude"] for wp in wp_max]
                    rx_max, ry_max, _ = proc.latlon_lists_to_utm(lat_max, lon_max)
                    route_max_xy = (rx_max, ry_max)
            # distance tolerance
            wp_dist = rs.simplify_by_distance_tolerance(self.args.route_dist_tol)
            if wp_dist:
                lat_d = [wp["latitude"] for wp in wp_dist]
                lon_d = [wp["longitude"] for wp in wp_dist]
                rx_d, ry_d, _ = proc.latlon_lists_to_utm(lat_d, lon_d)
                route_dist_xy = (rx_d, ry_d)
            # distance + bearing
            wp_db = rs.simplify_by_distance_and_bearing(self.args.route_dist_tol, self.args.route_angle_weight)
            if wp_db:
                lat_db = [wp["latitude"] for wp in wp_db]
                lon_db = [wp["longitude"] for wp in wp_db]
                rx_db, ry_db, _ = proc.latlon_lists_to_utm(lat_db, lon_db)
                route_dist_bearing_xy = (rx_db, ry_db)

        # compute smoothed fixed-wing path (use max_waypoints route if available, else filtered track)
        smoother = FixedWingPathSmoother(tension=0.5, points_per_segment=50)
        route_smooth_xy = None
        # choose base route: prefer max_waypoints, else dist_bearing, else dist, else filtered
        base = None
        if route_max_xy:
            base = route_max_xy
        elif route_dist_bearing_xy:
            base = route_dist_bearing_xy
        elif route_dist_xy:
            base = route_dist_xy
        else:
            # use filtered UTM points
            if xs_f and ys_f:
                base = (xs_f, ys_f)
        if base:
            bx, by = base
            if bx and by and len(bx) >= 2:
                sx, sy = smoother.smooth(list(bx), list(by))
                route_smooth_xy = (sx, sy)

        # export waypoints to QGroundControl .wpl files (one file per method)
        exporter = QGCWaypointsExporter()
        out_base = os.path.splitext(self.args.out)[0]
        try:
            if route_max_xy:
                exporter.write_wpl_from_utm(route_max_xy[0], route_max_xy[1], None, zone_info,
                                            f"{out_base}_max.waypoints")
            if route_dist_xy:
                exporter.write_wpl_from_utm(route_dist_xy[0], route_dist_xy[1], None, zone_info,
                                            f"{out_base}_dist.waypoints")
            if route_dist_bearing_xy:
                exporter.write_wpl_from_utm(route_dist_bearing_xy[0], route_dist_bearing_xy[1], None, zone_info,
                                            f"{out_base}_dist_bearing.waypoints")
            if route_smooth_xy:
                exporter.write_wpl_from_utm(route_smooth_xy[0], route_smooth_xy[1], None, zone_info,
                                            f"{out_base}_smoothed.waypoints")
        except Exception as e:
            print(f"Failed to write waypoint files: {e}")

        # plot original, filtered, removed and all route variants including smoothed route
        UTMPlotter.plot_all(xs, ys, times, xs_f, ys_f, times_f, removed, zone_info,
                            self.args.out, self.args.show,
                            route_max=route_max_xy,
                            route_dist=route_dist_xy,
                            route_dist_bearing=route_dist_bearing_xy,
                            route_smooth=route_smooth_xy)


def main():
    GPSApp().run()


if __name__ == "__main__":
    main()