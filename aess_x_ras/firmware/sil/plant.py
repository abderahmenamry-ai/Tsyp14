"""plant.py - simple physical models used around the firmware: DC motors + wheels + encoders + gyro,
and an LD06 packet generator. Used by both the quick behaviour tests and the PyBullet simulation."""
import math
import random
import struct

PKT_POINTS, DEG_PER_POINT = 12, 0.8
POINTS_PER_REV = 450


def crc8_ld06(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x4D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def ld06_sweep(point_fn, n_points=POINTS_PER_REV):
    """point_fn(angle_cw_deg) -> (distance_mm, intensity). Returns the list of 47-byte packets of one revolution."""
    pkts = []
    for k in range(0, n_points, PKT_POINTS):
        start = k * DEG_PER_POINT
        pts = []
        for i in range(PKT_POINTS):
            j = k + i
            pts.append(point_fn(j * DEG_PER_POINT) if j < n_points else (0, 0))
        end = (start + (PKT_POINTS - 1) * DEG_PER_POINT) % 360.0
        b = bytearray([0x54, 0x2C]) + struct.pack("<HH", 3600, int(round(start * 100)))
        for d, q in pts:
            b += struct.pack("<HB", int(d) & 0xFFFF, int(q) & 0xFF)
        b += struct.pack("<HH", int(round(end * 100)) & 0xFFFF, 0)
        b.append(crc8_ld06(bytes(b)))
        pkts.append(bytes(b))
    return pkts


class DrivePlant:
    """Two DC motors with first-order response, a little gain mismatch and encoders."""

    def __init__(self, wheel_d=0.065, wheel_base=0.30, tpr=374.0, max_speed=0.9, tau=0.08,
                 gain=(0.97, 1.03), enc_scale=1.01, stiction=0.08, gyro_bias_dps=0.05, gyro_noise_dps=1.58, seed=3):
        self.D, self.B, self.tpr, self.vmax, self.tau = wheel_d, wheel_base, tpr, max_speed, tau
        self.gain, self.enc_scale, self.stiction = gain, enc_scale, stiction
        self.v = [0.0, 0.0]
        self.acc = [0.0, 0.0]
        self.enc = [0, 0]
        self.bias, self.noise = gyro_bias_dps, gyro_noise_dps
        self.rng = random.Random(seed)
        self.dist = 0.0

    def step(self, duty_l, duty_r, dt):
        """Advance dt seconds. Returns (v_body, w_body)."""
        for i, d in enumerate((duty_l, duty_r)):
            target = 0.0 if abs(d) < self.stiction else d * self.vmax * self.gain[i]
            self.v[i] += (target - self.v[i]) * min(1.0, dt / self.tau)
            self.acc[i] += self.v[i] * dt / (math.pi * self.D) * self.tpr * self.enc_scale
            self.enc[i] = int(self.acc[i])
        v = 0.5 * (self.v[0] + self.v[1])
        w = (self.v[1] - self.v[0]) / self.B
        self.dist += abs(v) * dt
        return v, w

    def gyro_dps(self, w_body):
        return math.degrees(w_body) + self.bias + self.rng.gauss(0, self.noise)
