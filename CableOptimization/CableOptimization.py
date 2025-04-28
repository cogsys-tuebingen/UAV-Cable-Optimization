import math
import numpy as np


class CableOptimization():
    ## parameters
    drone_mass = 1710
    motor_count = 4
    flight_height = 30
    tether_voltage = 75

    # Aluminium
    conductor_specific_resistance = 0.0278  # in ohm mm^2/m
    conductor_density = 2.7  # in g/cm^3

    insulation_thickness = 0.75  # in mm
    insulation_density = 1.36  # in g/cm^3

    converter_efficiency = 0.9
    min_voltage = tether_voltage / 2
    payload_power = 15

    target_velocity = 30 / 3.6
    # last parameter w (wind) is ignored/set to zero
    drag_estimation_parameters = [1.45142232e-02, 1.36143191e-02, 1.92665906e+01, -1.22488489e+00]

    # constants

    tether_drag_coefficient = 0.75
    air_density = 1.293
    velocity = 0
    drag_coefficient = 1
    gravity = 9.80665

    interval = (0.0, 1.5)

    def set_conductor(self, material="aluminium"):
        if material == "aluminium" or material == "aluminum":
            self.conductor_specific_resistance = 0.0278
            self.conductor_density = 2.7
        elif material == "copper":
            self.conductor_specific_resistance = 0.0178
            self.conductor_density = 8.9

    def set_voltage(self, tether_voltage, converter_voltage):
        self.tether_voltage = tether_voltage
        self.min_voltage = max(converter_voltage, tether_voltage / 2)

    def set_motor_curve(self, motor_curve):
        self.motor_curve = motor_curve
        self.thrusts = np.array(motor_curve[0])
        self.powers = self.thrusts / np.array(motor_curve[1])
        self.thrusts = np.insert(self.thrusts, 0, 0)
        self.powers = np.insert(self.powers, 0, 0)
        self.thrusts = np.append(self.thrusts, self.thrusts[-1])
        self.powers = np.append(self.powers, np.inf)

        # calculate cross section upper bound at max power consumption
        max_power = self.motor_count * motor_curve[0][-1] / (
                motor_curve[1][-1] * self.converter_efficiency) + self.payload_power
        min_resistance = self.min_voltage * (self.tether_voltage - self.min_voltage) / max_power
        max_cross = 2 * self.flight_height * self.conductor_specific_resistance / min_resistance
        self.interval = (0.0, max_cross)

    def set_motor_curve_dict(self, motor_curve):
        motor_curve = motor_curve.copy()  # make a copy, so we don't modify the original dict
        if "name" in motor_curve:
            motor_curve.pop("name")
        sorted_values = sorted(motor_curve.values(), key=lambda x: x[0])
        thrusts, efficiencies = zip(*sorted_values)
        self.set_motor_curve([list(thrusts), list(efficiencies)])

    def effective_area(self, pitch, a, b):
        pitch = np.abs(pitch)
        return np.abs(a * np.cos(np.radians(pitch)) + b * np.sin(np.radians(pitch)))

    def drag_model(self, x, a, b, j, w=0):
        v, pitch = x
        v = v - w
        return self.air_density * self.drag_coefficient * self.effective_area(pitch, a, b) / 2 * (
                v ** 2 * np.sign(v) + j * v)

    def equilibrium_angle(self, target_velocity, drone_weight, additional_drag, *params):
        from scipy.optimize import root_scalar
        def equation(pitch):
            drag = self.drag_model((target_velocity, pitch), *params)
            # Force balance
            thrust_component = drone_weight * self.gravity * np.tan(np.radians(pitch))
            return drag + additional_drag - thrust_component

        try:
            interval = [-45, 45]  # Reasonable pitch angle in degrees
            result = root_scalar(equation, bracket=interval, method='brentq')
            return result.root
        except:
            return float('inf')

    def thrust_curve(self, thrust_point):
        return np.interp(thrust_point, self.thrusts, self.powers)

    def tether_resistance(self, conductor_cross):
        if conductor_cross <= 0:
            return float('inf')
        return 2 * self.flight_height * self.conductor_specific_resistance / conductor_cross

    def received_power(self, conductor_cross):
        return self.min_voltage * (self.tether_voltage - self.min_voltage) / self.tether_resistance(conductor_cross)

    def available_thrust(self, conductor_cross):
        return np.interp(
            self.converter_efficiency * (self.received_power(conductor_cross) - self.payload_power) / self.motor_count,
            self.powers,
            self.thrusts)

    def conductor_radius(self, conductor_cross):
        return math.sqrt(conductor_cross / math.pi)

    def tether_weight(self, conductor_cross):
        insulation_cross = math.pi * (
                self.insulation_thickness + self.conductor_radius(conductor_cross)) ** 2 - conductor_cross
        return 2 * self.flight_height * (
                self.conductor_density * conductor_cross + self.insulation_density * insulation_cross)

    def tether_drag(self, conductor_cross):
        area = 4 * self.flight_height * (self.conductor_radius(conductor_cross) + self.insulation_thickness) / 1000
        return 0.5 * area * self.air_density * self.tether_drag_coefficient * self.velocity ** 2

    def necessary_thrust(self, conductor_cross):
        weight = (self.drone_mass + self.tether_weight(conductor_cross)) / 1000
        additional_drag = self.tether_drag(conductor_cross)
        if self.velocity == 0:
            pitch = 0.0
        else:
            pitch = self.equilibrium_angle(self.velocity, weight, additional_drag,
                                           *self.drag_estimation_parameters[:-1])
        return 1000 * weight / (np.cos(np.radians(pitch)) * self.motor_count)

    def necessary_power(self, conductor_cross):
        return (1 / self.converter_efficiency) * (
                self.motor_count * (self.thrust_curve(self.necessary_thrust(conductor_cross)))) + self.payload_power

    def tether_current(self, conductor_cross, necessary_power):
        resistance = self.tether_resistance(conductor_cross)
        A = resistance
        B = - self.tether_voltage
        C = necessary_power

        discriminant = B ** 2 - 4 * A * C
        if discriminant < 0:
            return math.inf
        current = (-B - math.sqrt(discriminant)) / (2 * A)

        return current

    def total_power(self, conductor_cross):
        necessary_power = self.necessary_power(conductor_cross)
        return self.tether_current(conductor_cross, necessary_power) * self.tether_voltage

    def thrust_difference(self, conductor_cross):
        return self.available_thrust(conductor_cross) - 2 * self.necessary_thrust(conductor_cross)

    def get_solution(self):
        from scipy.optimize import root_scalar
        try:
            solution = root_scalar(self.thrust_difference, bracket=self.interval, method='brentq')
            return solution.root
        except:
            return None

    def get_minimum_power(self):
        from scipy.optimize import minimize_scalar
        try:
            # optimum can lie above bounds calculated from maximum power requirements due to power loss
            # but the power loss can't be bigger than the maximum power => uppwer bounds 2 * maximum power
            interval = [self.interval[0], 2 * self.interval[1]]
            result = minimize_scalar(self.total_power, bounds=interval, method='bounded')
            return result.x
        except:
            return None

    def set_hover(self, state):
        if state:
            self.velocity = 0
        else:
            self.velocity = self.target_velocity
