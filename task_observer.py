import math
from utime import ticks_us, ticks_diff

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

    # def __init__(self,
    #              left_encoder, right_encoder,
    #              imu_heading_share, imu_yaw_share,
    #              input_left_share, input_right_share,
    #              out_shares, statePredTime, statePredX, statePredY, statePredSL, stateMeasXL,
    #              Ts=0.03,
    #              Ad=None, Bd=None, Cd=None, Dd=None,
    #              publish_yhat=False,
    #              yhat_shares=None,
    #              imu_in_degrees=True):
    def __init__(self,
                 left_encoder, right_encoder,
                 imu_heading_share, imu_yaw_share,
                 input_left_share, input_right_share,
                 out_shares, statePredX, statePredY, leftMotorGo, rightMotorGo, sL_yhat, sL_meas, statetime,
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

        self.out = out_shares  # expects keys: 's','psi','omega_L','omega_R','X','Y'
        if Ts <= 0:
            Ts = 0.03
        self.Ts = Ts
        self.Ts_nom = Ts          # nominal sample time (e.g. 0.03)
        self._last_t_us = ticks_us()
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
        # self._statePredTime = statePredTime
        self._statePredX = statePredX
        self._statePredY = statePredY
        self._leftMotorGo = leftMotorGo
        self._rightMotorGo = rightMotorGo
        self._sL_yhat = sL_yhat
        self._sL_meas = sL_meas
        self._statetime = statetime
        # self._statePredSL = statePredSL
        # self._stateMeasXL = stateMeasXL
        # infer dimensions
        self.nx = len(self.Ad)
        if self.nx == 0 or len(self.Ad[0]) != self.nx:
            raise ValueError("Ad must be square (nx x nx).")

        self.nu_star = len(self.Bd[0])
        if len(self.Bd) != self.nx:
            raise ValueError("Bd must be (nx x nu_star).")

        # state estimate
        self.xhat = [0.0] * self.nx
        self.xhatPrev = 0
        # Global position states (not part of observer matrix)
        self.X = 0.0
        self.Y = 0.0
        self.idx = 0
        self.sim_time = 0

        # Sample time (must match discretization used for Ad/Bd)
        # self.Ts = 0.03 # <-- replace with actual sampling period


        # For the assignment, y has 4 elements: [xL, xR, psi, omega]
        self.ny = 4

        # We expect u_star = [uL, uR, y...] -> length 2 + 4 = 6
        # If your MATLAB export differs, we'll still build u_star to match Bd width.
        # But by default we build the assignment ordering.
        # You can adjust the packing in read_inputs() if needed.
    
    def read_inputs(self):
        """Read encoder + IMU + inputs, return (u_star, y_meas_dict)."""

        # Encoder measurements
        self.left_enc.update()
        self.right_enc.update()
        xL = self.left_enc.get_position()
        xR = self.right_enc.get_position()
        vL = self.left_enc.get_velocity()
        vR = self.right_enc.get_velocity()

        if not self._sL_meas.full():
            self._sL_meas.put(xL)

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
            uL = float(self.sh_uL.get()) * 7.2 / 100 # volts
            uR = float(self.sh_uR.get()) * 7.2 / 100 # volts
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

        # Extract estimated velocity and heading
        #v = self.xhat[1] # make sure index matches your state order
        psi = self.xhat[1]

        now = ticks_us()
        dt = ticks_diff(now, self._last_t_us) / 1_000_000.0
        self.sim_time += dt
        self._last_t_us = now

        # Optional: clamp dt so one glitch doesn't explode your integration
        if dt <= 0 or dt > 5*self.Ts_nom:
            dt = self.Ts_nom
        
        v = (self.xhat[0]-self.xhatPrev)/dt
        self.xhatPrev = self.xhat[0]
        

        # Dead-reckoning integration
        # self.X += self.Ts * v * math.cos(psi)
        # self.Y += self.Ts * v * math.sin(psi)
        self.X += dt * v * math.cos(psi)
        self.Y += dt * v * math.sin(psi)
        self.idx += 1
        # if self.idx == 10:
        #     if not self._statePredX.full():
        #         self._statetime.put(self.sim_time)
        #         self._statePredX.put(self.X)
        #         self._statePredY.put(self.Y)
        #         # self.idx = 0 


    def compute_yhat(self, u_star):
        """Optional estimated output yhat = Cd*xhat + Dd*u_star"""
        if self.Cd is None or self.Dd is None:
            return None
        Cx = dot_mv(self.Cd, self.xhat)
        Du = dot_mv(self.Dd, u_star)
        self.yhat = vec_add(Cx, Du)
        # if self.idx == 10:  # new
        #     if not self._sL_yhat.full():
        #         self._sL_yhat.put(self.yhat[0])
        #         self.idx = 0 # new
        #     return self.yhat

    def publish(self):
        """Publish state estimate to shares using the assignment’s state ordering:
           xhat = [x, v, psi, omega] (adjust if your state ordering differs!)
        """
        # If your MATLAB state ordering is different, change these indices.
        try:
            self.out['s'].put(self.xhat[0])
            self.out['psi'].put(normalize_angle_rad(self.xhat[1]))
            self.out['omega_L'].put(self.xhat[2])
            self.out['omega_R'].put(self.xhat[3])
            if 'X' in self.out:
                self.out['X'].put(self.X)
            if 'Y' in self.out:
                self.out['Y'].put(self.Y)
        except Exception:
            pass

    def run(self):
        while True:
            try:
                u_star, meas = self.read_inputs()
                
                self.step(u_star)
                #self._stateMeasXL.put(meas['xL'])
                #if self._leftMotorGo.get() and self._rightMotorGo.get():
                    #print(f"xhat:\n\r s:{self.xhat[0]} psi:{self.xhat[1]} omega_L:{self.xhat[2]}, omega_R:{self.xhat[3]}\n\r X:{self.X}, Y:{self.Y}\n\r\r")
                
                self.publish()
                #self._statePredX.put(self.X)
                #self._statePredY.put(self.Y)
# self._statePredTime.put( idk man insert some tickdiff stuff :)
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
                        #self._statePredSL.put(yhat[3])
            except Exception as e:
                # Don’t let the task silently die — print once per failure
                print("Observer error:", e)

            yield 0