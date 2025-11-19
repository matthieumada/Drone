# ...existing code...
import numpy as np
import matplotlib.pyplot as plt
from utm import utmconv
from hermite import cubic_hermite_spline

class PathSmoother:
    """
    Class to smooth trajectory UTM (easting, northing) or
    dtuples  UTM full (hem, zone, letter, easting, northing).
    """
    def __init__(self, data, angle_thresh_deg=70.0, steps=20):
        self.raw = data
        self.angle_thresh_deg = angle_thresh_deg
        self.steps = steps
        self.uc = utmconv()
        self.chs = cubic_hermite_spline()
        self.pts = [self._utm_to_xy(p) for p in data]
        self.traj = []

    def _utm_to_xy(self, item):
        """Return (easting, northing) float from item UTM or (easting,northing)."""
        if len(item) >= 5:
            _, _, _, e, n = item[:5]
            return (float(e), float(n))
        elif len(item) == 2:
            return (float(item[0]), float(item[1]))
        else:
            raise ValueError("Unsupported point format: {}".format(item))

    def _angle_between(self, v1, v2):
        """Angle en degrés entre v1 et v2."""
        a = np.array(v1, dtype=float)
        b = np.array(v2, dtype=float)
        na = np.linalg.norm(a)
        nb = np.linalg.norm(b)
        if na == 0 or nb == 0:
            return 0.0
        cosang = np.clip(np.dot(a, b) / (na * nb), -1.0, 1.0)
        return np.degrees(np.arccos(cosang))

    def smooth(self):
        """Caompute and return the smooth trajectory  (easting,northing))."""
        pts = self.pts
        if len(pts) < 2:
            self.traj = pts.copy()
            return self.traj

        traj = []
        traj.append(pts[0])

        for i in range(1, len(pts) - 1):
            p_prev = np.array(pts[i - 1], dtype=float)
            p_cur = np.array(pts[i], dtype=float)
            p_next = np.array(pts[i + 1], dtype=float)

            v1 = p_cur - p_prev
            v2 = p_next - p_cur

            ang = self._angle_between(v1, v2)
            if ang >= self.angle_thresh_deg:
                t1 = v1.tolist()
                t2 = v2.tolist()
                interp = self.chs.goto_wpt(p_cur.tolist(), t1, p_next.tolist(), t2, self.steps)
                for pt in interp[1:]:
                    traj.append((float(pt[0]), float(pt[1])))
            else:
                traj.append((float(p_cur[0]), float(p_cur[1])))

        traj.append(pts[-1])
        self.traj = traj
        return traj

    def plot(self, show=True, savepath=None):
        """First and new trajecotry """
        if not self.traj:
            self.smooth()
        ox, oy = zip(*self.pts)
        tx, ty = zip(*self.traj)
        plt.figure()
        plt.plot(ox, oy, 'o--', label='original')
        plt.plot(tx, ty, 'r-', label='smoothed')
        plt.axis('equal')
        plt.legend()
        if savepath:
            plt.savefig(savepath, dpi=150)
        if show:
            plt.show()
        # plt.close()

if __name__ == "__main__":
    sample = [
        (500000.0, 0.0),
        (500100.0, 0.0),
        (500100.0, 100.0),  # coin franc à 90 deg
        (500200.0, 100.0)
    ]
    smoother = PathSmoother(sample, angle_thresh_deg=45.0, steps=30)
    out = smoother.smooth()
    print("Trajectory points:", len(out))
    smoother.plot(show=True, savepath='comparison')
# ...existing code...