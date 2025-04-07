class DragEstimation:
    air_density = 1.293
    drag_coefficient = 1
    gravity = 9.80665

    def load(self, file_path):
        from pymavlink import mavutil

        mav = mavutil.mavlink_connection(file_path)

        self.data = []
        while True:
            msg = mav.recv_msg()
            if msg is None:
                break
            else:
                if msg.get_type() == "XKF1":
                    if msg.C == 0:  # only take core 0 EKF
                        self.data.append({
                            'time': msg.TimeUS,
                            'pitch': msg.Pitch,
                            'roll': msg.Roll,
                            'yaw': msg.Yaw,
                            'VE': msg.VE,
                            'VN': msg.VN,
                            'VD': msg.VD,
                        })

    def process_data_s(self, start_time, end_time, pitch_offset=0):
        min_time = self.data[0]['time'] / 1000000
        max_time = self.data[-1]['time'] / 1000000
        duration = max_time - min_time
        self.process_data((start_time - min_time) / duration, (end_time - min_time) / duration, pitch_offset)

    def process_data(self, start=0.0, end=1.0, pitch_offset=0.0):
        import pandas as pd
        import numpy as np
        from scipy.signal import savgol_filter

        # Create DataFrame
        self.df = pd.DataFrame(self.data)

        # Slice the DataFrame
        slice_start = max(int(len(self.df) * start), 0)
        slice_end = min(int(len(self.df) * end), len(self.df))
        self.df = self.df.iloc[slice_start:slice_end]

        # apply pitch offset
        self.df['pitch'] = self.df['pitch'] - pitch_offset

        # do trigonometry
        # sin_pitch = np.sin(np.radians(self.df['pitch']))
        sin_yaw = np.sin(np.radians(self.df['yaw']))
        # cos_pitch = np.cos(np.radians(self.df['pitch']))
        cos_roll = np.cos(np.radians(self.df['roll']))
        cos_yaw = np.cos(np.radians(self.df['yaw']))
        tan_pitch = np.tan(np.radians(self.df['pitch']))

        # calculate stabilized frame velocity
        self.df['VX'] = cos_yaw * self.df['VN'] + sin_yaw * self.df['VE']

        # calculate observed acceleration in stabilized body frame
        # forward direction
        self.df['AX_o'] = np.gradient(self.df['VX'], self.df['time'] / 1000000)
        # downward direction
        self.df['AD_o'] = np.gradient(self.df['VD'], self.df['time'] / 1000000)

        # calculate acceleration from pitch
        # thrust_a = (-DragEstimation.gravity + self.df['AD_o']) / (cos_pitch * cos_roll)
        # self.df['AX_e'] = thrust_a * sin_pitch
        self.df['AX_e'] = (-DragEstimation.gravity + self.df['AD_o']) * tan_pitch / cos_roll

        # calculate drag
        self.df['DX'] = self.df['AX_e'] - self.df['AX_o']

        # invert pitch so the sign matches the acceleration
        self.df['pitch'] = -self.df['pitch']

        self.velocities = np.array(self.df['VX'].dropna())
        self.drags = np.array(self.df['DX'].dropna())
        self.pitches = np.array(self.df['pitch'].dropna())

        self.smoothed_drags = savgol_filter(self.drags, 23, 1)

        self.noise = self.smoothed_drags - self.drags
        self.smoothed_noise = savgol_filter(np.abs(self.noise), 23, 1)
        self.noise_mean = np.mean(self.noise)
        self.noise_stdev = np.std(self.noise, mean=self.noise_mean)

    def effective_area(self, pitch, Af, At):
        import numpy as np

        pitch = np.abs(pitch)
        return Af * np.cos(np.radians(pitch)) + At * np.sin(np.radians(pitch))

    def drag_model(self, x, Af, At, j, w):
        import numpy as np

        v, pitch = x
        v = v - w
        return DragEstimation.air_density * DragEstimation.drag_coefficient * self.effective_area(pitch, Af, At) / 2 * (
                v ** 2 * np.sign(v) + j * v)

    def fit(self, drone_mass):
        from scipy.optimize import curve_fit
        import numpy as np

        self.drone_mass = drone_mass

        initial_guess = [1.0, 1.0, 1.0, 1.0]
        self.popt, self.pconv = curve_fit(self.drag_model,
                                          (self.velocities, self.pitches),
                                          self.drags * drone_mass,
                                          initial_guess)
        self.pstdev = np.sqrt(np.diag(self.pconv))
        self.pcvar = self.pstdev / (np.abs(self.popt) + 1e-12)
        return self.popt

    def equilibrium_angle(self, target_velocity, drone_weight, *params):
        import numpy as np
        from scipy.optimize import root_scalar
        def equation(pitch):
            drag = self.drag_model((target_velocity, pitch), *params)
            # Force balance
            thrust_component = drone_weight * self.gravity * np.tan(np.radians(pitch))
            return drag - thrust_component

        interval = [-45, 45]  # Reasonable pitch angle in degrees
        result = root_scalar(equation, bracket=interval, method='brentq')
        return result.root


async def estimate(file_path, drone_mass, start=0.0, end=1.0, pitch_offset=0.0):
    drag = DragEstimation()
    drag.load(file_path)
    drag.process_data(start, end, pitch_offset)
    drag.fit(drone_mass)
    return drag.popt
