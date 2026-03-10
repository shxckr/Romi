import math

# -----------------------------
# Tiny helpers: matrix-vector ops
# -----------------------------
def dot_mv(M, v):
    """Matrix-vector multiply. M: list-of-lists (r x c), v: list (c). Returns list (r)."""
    out = []
    for row in M:
        s = 0.0
        # zip truncates to shorter length; we WANT a dimension check instead
        if len(row) != len(v):
            raise ValueError("dot_mv dim mismatch: row has {}, v has {}".format(len(row), len(v)))
        for a, b in zip(row, v):
            s += a * b
        out.append(s)
    return out

def vec_add(a, b):
    if len(a) != len(b):
        raise ValueError("vec_add dim mismatch: {} vs {}".format(len(a), len(b)))
    return [ai + bi for ai, bi in zip(a, b)]

def normalize_angle_rad(theta):
    """Normalize to [-pi, pi)."""
    return (theta + math.pi) % (2 * math.pi) - math.pi


class task_observer:
    """
    Discrete-time observer task (MATLAB augmented-input form):

        xhat[k+1] = Ad * xhat[k] + Bd * u_star[k]
        u_star = [uL, uR, y0, y1, y2, y3]^T
              = [uL, uR, xL, xR, psi, omega]^T   (per assignment)

    Optional (for debug/plotting):
        yhat[k] = Cd * xhat[k] + Dd * u_star[k]

    Notes:
    - Units must match your MATLAB model.
    - If MATLAB used radians, you MUST convert IMU deg -> rad here (default does).
    - Encoders should be in meters (or radians) consistently with MATLAB.
    """

    def __init__(self,
                 left_encoder, right_encoder,
                 imu_heading_share, imu_yaw_share,
                 input_left_share, input_right_share,
                 out_shares,
                 Ts=0.03,
                 Ad=None, Bd=None, Cd=None, Dd=None,
                 publish_yhat=False,
                 yhat_shares=None,
                 imu_in_degrees=True):
        self.left_enc = left_encoder
        self.right_enc = right_encoder

        self.sh_heading = imu_heading_share
        self.sh_yawrate = imu_yaw_share

        self.sh_uL = input_left_share
        self.sh_uR = input_right_share

        self.out = out_shares  # expects keys: 'x','v','psi','omega'

        self.Ts = Ts
        self.imu_in_degrees = imu_in_degrees

        if Ad is None or Bd is None:
            raise ValueError("Ad and Bd must be provided (precomputed offline).")

        self.Ad = Ad
        self.Bd = Bd

        # Optional output model (debug)
        self.Cd = Cd
        self.Dd = Dd
        self.publish_yhat = publish_yhat
        self.yhat_shares = yhat_shares or {}

        # infer dimensions
        self.nx = len(self.Ad)
        if self.nx == 0 or len(self.Ad[0]) != self.nx:
            raise ValueError("Ad must be square (nx x nx).")

        self.nu_star = len(self.Bd[0])
        if len(self.Bd) != self.nx:
            raise ValueError("Bd must be (nx x nu_star).")

        # state estimate
        self.xhat = [0.0] * self.nx

        # For the assignment, y has 4 elements: [xL, xR, psi, omega]
        self.ny = 4

        # We expect u_star = [uL, uR, y...] -> length 2 + 4 = 6
        # If your MATLAB export differs, we'll still build u_star to match Bd width.
        # But by default we build the assignment ordering.
        # You can adjust the packing in read_inputs() if needed.

    def read_inputs(self):
        """Read encoder + IMU + inputs, return (u_star, y_meas_dict)."""

        # Encoder measurements
        xL = self.left_enc.get_position()
        xR = self.right_enc.get_position()
        vL = self.left_enc.get_velocity()
        vR = self.right_enc.get_velocity()

        # IMU measurements
        try:
            heading = float(self.sh_heading.get())
            yawrate = float(self.sh_yawrate.get())
        except Exception:
            heading = 0.0
            yawrate = 0.0

        if self.imu_in_degrees:
            psi = math.radians(heading)
            omega = math.radians(yawrate)
        else:
            psi = heading
            omega = yawrate

        # Inputs (motor commands)
        try:
            uL = float(self.sh_uL.get())
            uR = float(self.sh_uR.get())
        except Exception:
            uL = 0.0
            uR = 0.0

        # Build measurement vector y = [xL, xR, psi, omega]
        y = [xL, xR, psi, omega]

        # Build augmented input u_star = [uL, uR, y...]
        u_star = [uL, uR] + y

        # If Bd width isn't 6, trim/pad cautiously (so it won't crash)
        if len(u_star) > self.nu_star:
            u_star = u_star[:self.nu_star]
        elif len(u_star) < self.nu_star:
            u_star = u_star + [0.0] * (self.nu_star - len(u_star))

        meas = {
            'xL': xL, 'xR': xR, 'vL': vL, 'vR': vR,
            'psi': psi, 'omega': omega,
            'uL': uL, 'uR': uR
        }

        return u_star, meas

    def step(self, u_star):
        """One observer update: xhat <- Ad*xhat + Bd*u_star"""
        Ax = dot_mv(self.Ad, self.xhat)
        Bu = dot_mv(self.Bd, u_star)
        self.xhat = vec_add(Ax, Bu)

    def compute_yhat(self, u_star):
        """Optional estimated output yhat = Cd*xhat + Dd*u_star"""
        if self.Cd is None or self.Dd is None:
            return None
        Cx = dot_mv(self.Cd, self.xhat)
        Du = dot_mv(self.Dd, u_star)
        return vec_add(Cx, Du)

    def publish(self):
        """Publish state estimate to shares using the assignment’s state ordering:
           xhat = [x, v, psi, omega] (adjust if your state ordering differs!)
        """
        # If your MATLAB state ordering is different, change these indices.
        try:
            self.out['x'].put(self.xhat[0])
            self.out['v'].put(self.xhat[1])
            self.out['psi'].put(normalize_angle_rad(self.xhat[2]))
            self.out['omega'].put(self.xhat[3])
        except Exception:
            pass

    def run(self):
        while True:
            try:
                u_star, meas = self.read_inputs()
                self.step(u_star)
                self.publish()

                # Optional yhat publishing for comparison/plotting
                if self.publish_yhat:
                    yhat = self.compute_yhat(u_star)
                    if yhat is not None:
                        # expected yhat ordering: [xL_hat, xR_hat, psi_hat, omega_hat]
                        # publish only if shares exist
                        sh = self.yhat_shares
                        if 'xL_hat' in sh: sh['xL_hat'].put(yhat[0])
                        if 'xR_hat' in sh: sh['xR_hat'].put(yhat[1])
                        if 'psi_hat' in sh: sh['psi_hat'].put(normalize_angle_rad(yhat[2]))
                        if 'omega_hat' in sh: sh['omega_hat'].put(yhat[3])

            except Exception as e:
                # Don’t let the task silently die — print once per failure
                print("Observer error:", e)

            yield 0