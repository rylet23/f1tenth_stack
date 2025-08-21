import numpy as np
from math import radians
from sensor_msgs.msg import PointCloud2


class Camera:
    def __init__(self, node, topic='/zed2i/zed_node/point_cloud/cloud',forward_axis='z', max_range=20.0):
        self.node = node
        self.topic = topic
        self.forward_axis = forward_axis
        self.maxr = float(max_range)
        self._off = None
        self.last = None

        node.create_subscription(PointCloud2, topic, self._cb, 10)

    def _cb(self, msg: PointCloud2):
        try:
            if self._off is None:
                d = {f.name: f for f in msg.fields}
                self._off = (d['x'].offset, d['y'].offset, d['z'].offset)

            be = '>f4' if msg.is_bigendian else '<f4'
            step = msg.point_step
            buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, step)

            x = buf[:, self._off[0]:self._off[0]+4].view(be).ravel()
            y = buf[:, self._off[1]:self._off[1]+4].view(be).ravel()
            z = buf[:, self._off[2]:self._off[2]+4].view(be).ravel()

            if self.forward_axis == 'x':
                fwd, lat = x, y
            else:
                fwd, lat = z, x

            r = np.hypot(fwd, lat)
            a = np.arctan2(lat, fwd)

            m = np.isfinite(r) & (r > 0.05) & (r < self.maxr)
            self.last = (r[m], a[m])

        except Exception as e:
            try:
                self.node.get_logger().error(f'CloudHelper: {e}')
            except Exception:
                pass

    def sector_stats(self, front_half_deg=15, side_deg=45, side_half_deg=20):
        if self.last is None:
            return float('inf'), float('inf'), float('inf')

        r, a = self.last

        def pick(center, half):
            h = radians(half); c = radians(center)
            m = (a >= c - h) & (a <= c + h)
            if not np.any(m):
                return float('inf'), float('inf')
            vals = r[m]
            if vals.size == 0:
                return float('inf'), float('inf')
            return float(vals.min()), float(vals.mean())

        fmin, _  = pick(0.0, front_half_deg)
        _, lmean = pick(+side_deg, side_half_deg)
        _, rmean = pick(-side_deg, side_half_deg)

        return fmin, lmean, rmean
