import numpy as np
from math import sin, cos, atan, sqrt, atan2, copysign
import sys

class Rocket:
    def __init__(self, coord, euler, speed, d_t):
        self._coord = np.array(coord, dtype=np.float)  # All vectors in form [X, Y, Z]
        self._overload = np.array([0, 1, 0], dtype=np.float)
        self._euler = np.array(euler, dtype=np.float)  # [Theta, Psi, Gamma]
        self._speed = speed
        self._target = None
        self._n_y_max = 20
        self._d_t = d_t
        self._limit_angle = np.deg2rad(60)
        self._current_overloads = np.zeros(2)
        self._previous_distance_to_target = sys.float_info.max
        self._explotion_distance = 100

    def captureTarget(self, target):
        self._target = target

    @property
    def targetInfo(self):
        if self._target is not None:
            return self._target.state
        else:
            return None

    @property
    def state(self):
        """ coord, euler, speed, overload """
        return [np.copy(self._coord),
                np.copy(self._euler),
                self._speed,
                np.copy(self._overload),
                np.copy(self._current_overloads)]

    def step(self, action):
        """ action: np.array 1x2 [Nz, Ny] """
        self.update(action)

    def grav_compensate(self):

        self._euler[2] = 0
        compens = np.array([0, 1, 0])
        compens = toSpeedCoordinateSystem(self._euler, compens)
        
        compens[0] = 0

        self._overload = compens

    def sumOverloads(self, n_pitch, n_roll):
        n_roll += self._overload[2]
        self._current_overloads = np.array([n_roll, n_pitch])

        self._overload[1] += n_pitch
        if np.isclose(self._overload[1], 0):
            a = 1
            
        if not np.isclose(n_roll, 0):
            if np.isclose(self._overload[1], 0):
                self._euler[2] += np.pi/2 * np.sign(n_roll)
            else:
                self._euler[2] += atan(n_roll / self._overload[1]) 
            
        # self._euler[2] += 0 if np.isclose(n_roll, 0) else atan(n_roll / self._overload[1])
        self._overload[1] = sqrt(pow(self._overload[1], 2) + pow(n_roll, 2)) * (1 if self._overload[1] > 0 else -1)
        self._overload[2] = 0

    def TargetSpeed(self):

        target_speed = np.array([self.targetInfo[2] * cos(self.targetInfo[1][0]) * cos(self.targetInfo[1][1]),
                                 self.targetInfo[2] * sin(self.targetInfo[1][0]),
                                 -self.targetInfo[2] * cos(self.targetInfo[1][0]) * sin(self.targetInfo[1][1])])

        target_speed = toSpeedCoordinateSystem(self._euler, target_speed)

        return target_speed

    def CalculateNyPN(self, k_y=5):

        euler = np.copy(self._euler)
        euler[2] = 0
        target_coor = toSpeedCoordinateSystem(euler, self.targetInfo[0] - self._coord)
        target_speed = self.TargetSpeed()

        target_speed_xy = np.array([sqrt(pow(target_speed[0], 2) + pow(target_speed[1], 2)),
                                    atan2(target_speed[1], target_speed[0])])

        sigma_R_XY = -atan2(target_coor[1], target_coor[0])
        sigma_T_XY = target_speed_xy[1] - atan2(target_coor[1], target_coor[0])

        r_XY = sqrt(pow(target_coor[0], 2) + pow(target_coor[1], 2))

        d_lambda_XY = (target_speed_xy[0] * sin(sigma_T_XY) - self._speed * sin(sigma_R_XY)) / r_XY

        W_XY = k_y * self._speed * d_lambda_XY
        n_pitch = W_XY / G

        return n_pitch

    def CalculateNzPN(self, k_z=5):

        euler = np.copy(self._euler)
        euler[2] = 0
        TargetCoor = toSpeedCoordinateSystem(euler, self.targetInfo[0] - self._coord)
        TargetSpeed = self.TargetSpeed()

        TargetSpeedXZ = np.array([sqrt(pow(TargetSpeed[0], 2) + pow(TargetSpeed[2], 2)),
                                  atan2(-TargetSpeed[2], TargetSpeed[0])])

        sigma_R = -atan2(-TargetCoor[2], TargetCoor[0])
        sigma_T = TargetSpeedXZ[1] - atan2(-TargetCoor[2], TargetCoor[0])
        r = sqrt(pow(TargetCoor[0], 2) + pow(TargetCoor[2], 2))
        d_lambda = (TargetSpeedXZ[0] * sin(sigma_T) - self._speed * sin(sigma_R)) / r
        W = -k_z * self._speed * d_lambda
        n_roll = W / G

        return n_roll

    def proportionalCoefficients(self, k_z=5, k_y=5):
        """ WARNING: call only after grav_compensate shit code """
        return np.array([self.CalculateNzPN(k_z), self.CalculateNyPN(k_y)])

    def LimitOverload(self):
        self._overload[1] = (self._n_y_max - 1) * (1 if self._overload[1] > 0 else -1) if abs(
            self._overload[1]) > (self._n_y_max - 1) else self._overload[1]

    def update(self, action):

        self.grav_compensate()

        self.sumOverloads(action[1], action[0])

        self.LimitOverload()

        self.integrate(self._d_t)

    def integrate(self, dt):
        self._speed += (self._overload[0] - sin(self._overload[0])) * G * dt
        self._euler[0] += (self._overload[1] * cos(self._euler[2]) - cos(self._euler[0])) * G / self._speed * dt
        self._euler[1] += -self._overload[1] * sin(self._euler[2]) * G / (self._speed * cos(self._euler[0])) * dt
        self._coord[0] += self._speed * cos(self._euler[0]) * cos(self._euler[1]) * dt
        self._coord[1] += self._speed * sin(self._euler[0]) * dt
        self._coord[2] += -self._speed * cos(self._euler[0]) * sin(self._euler[1]) * dt

    @property
    def destroyed(self):
        if self.distanceToTarget < self._explotion_distance:
            return True
            # Closest distance
            # if np.greater_equal(np.round(self._previous_distance_to_target - self.distanceToTarget, 3), 0):
            #     self._previous_distance_to_target = self.distanceToTarget
            # else:
            #     return True

        return False

    def targetLost(self):

        target_coor = self.targetInfo[0]

        angle = np.arccos(abs(np.dot(target_coor, self._coord) /
                              (np.linalg.norm(target_coor) * np.linalg.norm(self._coord))))

        if angle > self._limit_angle:
            return True
        return False

    @property
    def distanceToTarget(self):
        return np.linalg.norm(self.targetInfo[0] - self._coord)


class LA:
    def __init__(self, coord, euler, speed, d_t):
        self._coord = np.array(coord, dtype=np.float)
        self._speed = speed
        self._euler = np.array(euler, dtype=np.float)
        self._overload = np.array([0, 1, 0], dtype=np.float)
        self._d_t = d_t
        self.t = 0

    @property
    def state(self):
        """ coord, euler, speed, overload """
        return [np.copy(self._coord),
                np.copy(self._euler),
                self._speed,
                np.copy(self._overload)]

    def integrate(self, dt):
        self._speed += (self._overload[0] - sin(self._overload[0])) * G * dt
        self._euler[0] += (self._overload[1] * cos(self._euler[2]) - cos(self._euler[0])) * G / self._speed * dt
        self._euler[1] += -self._overload[1] * sin(self._euler[2]) * G / (self._speed * cos(self._euler[0])) * dt
        self._coord[0] += self._speed * cos(self._euler[0]) * cos(self._euler[1]) * dt
        self._coord[1] += self._speed * sin(self._euler[0]) * dt
        self._coord[2] += -self._speed * cos(self._euler[0]) * sin(self._euler[1]) * dt

    def step(self):
        manouver_map = self.manouver(0)
        self.grav_compensate()

        overload = 0

        for index, obj in enumerate(manouver_map):
            if self.t < manouver_map[0,0]:
                break

            if manouver_map[index, 0]< self.t < manouver_map[index + 1,0]:
                overload = obj[1]
                break

        self.sumOverloads(0, overload)

        self.integrate(self._d_t)
        self.t += self._d_t

    def grav_compensate(self):
        self._euler[2] = 0
        compens = np.array([0, 1, 0])
        compens = toSpeedCoordinateSystem(self._euler, compens)

        self._overload = compens

    def sumOverloads(self, n_pitch, n_roll):
        self._current_overloads = np.array([n_roll, n_pitch])

        self._overload[1] += n_pitch
        self._euler[2] += 0 if np.isclose(n_roll, 0) else atan(n_roll / self._overload[1])
        self._overload[1] = sqrt(pow(self._overload[1], 2) + pow(n_roll, 2)) * (1 if self._overload[1] > 0 else -1)

    def manouver(self, type=0):
        if type == 0:
            start_sec = 5
            end_sec = 1000
            each_sec = 5
            nz = 3
            time_stamps = 3 + np.arange(np.ceil((end_sec-start_sec)/each_sec)+1)*each_sec
            overloads = np.tile(np.array([nz,-nz]), time_stamps.shape[0])[:time_stamps.shape[0]]
            return np.array([np.array([time_stamps[i], overloads[i]]) for i in range(time_stamps.shape[0])])

        return 0


G = 9.81


def toSpeedCoordinateSystem(euler, vec):
    if isinstance(euler, list):
        euler = np.array(euler)

    if isinstance(vec, list):
        vec = np.array(vec)

    euler = euler.reshape((1, 3))
    vec = vec.reshape((1, 3))

    matr = np.array([[cos(-euler[0, 0]) * cos(-euler[0, 1]), sin(-euler[0, 0]), -cos(-euler[0, 0]) * sin(-euler[0, 1])],

                     [-cos(-euler[0, 2]) * sin(-euler[0, 0]) * cos(-euler[0, 1]) + sin(-euler[0, 2]) * sin(
                         -euler[0, 1]),
                      cos(-euler[0, 2]) * cos(-euler[0, 0]),
                      cos(-euler[0, 2]) * sin(-euler[0, 0]) * sin(-euler[0, 1]) + sin(-euler[0, 2]) * cos(
                          -euler[0, 1])],

                     [sin(-euler[0, 2]) * sin(-euler[0, 0]) * cos(-euler[0, 1]) + cos(-euler[0, 2]) * sin(-euler[0, 1]),
                      -sin(-euler[0, 2]) * cos(-euler[0, 0]),
                      -sin(-euler[0, 1]) * sin(-euler[0, 0]) * sin(-euler[0, 2]) + cos(-euler[0, 1]) * cos(
                          -euler[0, 2])]
                     ])

    return np.dot(vec, matr).reshape(3)


def toTrajectoryCoordinateSystem(euler, vec):
    if isinstance(euler, list):
        euler = np.array(euler)

    if isinstance(vec, list):
        vec = np.array(vec)

    vec = vec.reshape((1, 3))
    euler = euler.reshape((1, 3))

    matr = np.array([
        [cos(euler[0, 0]) * cos(euler[0, 1]), sin(euler[0, 0]), -cos(euler[0, 0]) * sin(euler[0, 1])],
        [-cos(euler[0, 2]) * sin(euler[0, 0]) * cos(euler[0, 1]) + sin(euler[0, 2]) * sin(euler[0, 1]),
         cos(euler[0, 2]) * cos(euler[0, 0]),
         cos(euler[0, 2]) * sin(euler[0, 0]) * sin(euler[0, 1]) + sin(euler[0, 2]) * cos(euler[0, 1])],
        [sin(euler[0, 2]) * sin(euler[0, 0]) * cos(euler[0, 1]) + cos(euler[0, 2]) * sin(euler[0, 1]),
         -sin(euler[0, 2]) * cos(euler[0, 0]),
         -sin(euler[0, 1]) * sin(euler[0, 0]) * sin(euler[0, 2]) + cos(euler[0, 1]) * cos(euler[0, 2])]
    ])

    return np.dot(vec, matr).reshape(3)


def nz_approx(x):
    sign = copysign(1, x)
    if -10 <= x <= 10:
        return 1 / 2 * x
    elif -17 <= x <= 17:
        return (-15 + 2 * abs(x)) * sign
    else:
        return 20 * sign


ran = np.arange(-18, 19)
ALL_NZ = np.array([nz_approx(x) for x in ran])
ACTION_MEANING = {np.where(ran == x)[0][0]: f"Nz = {nz_approx(x):.2f}" for x in ran}

temp = np.array(np.meshgrid(ALL_NZ, ALL_NZ)).T.reshape(-1, 2)
norms = np.linalg.norm(temp, axis=1)
ALL_POSSIBLE_ACTIONS = temp[norms <= 20]
