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
        self._t = 0
        self._max_time = 60

        self.m = 157-51.01
        self.s = 0.4

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
        return np.hstack([np.copy(self._coord),
                np.copy(self._euler),
                self._speed,
                np.copy(self._overload),
                np.copy(self._current_overloads),
                np.copy(self.dataForNzPN()),
                self.angleToTarget])

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

    def TargetSpeed(self, angles):

        target_speed = np.array([self.targetInfo[6] * cos(self.targetInfo[3:6][0]) * cos(self.targetInfo[3:6][1]),
                                 self.targetInfo[6] * sin(self.targetInfo[3:6][0]),
                                 -self.targetInfo[6] * cos(self.targetInfo[3:6][0]) * sin(self.targetInfo[3:6][1])])

        
        target_speed = toSpeedCoordinateSystem(angles, target_speed)

        return target_speed

    def CalculateNyPN(self, k_y=5):

        euler = np.copy(self._euler)
        euler[2] = 0
        target_coor = toSpeedCoordinateSystem(euler, self.targetInfo[0:3] - self._coord)
        target_speed = self.TargetSpeed(self._euler)

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
        TargetCoor = toSpeedCoordinateSystem(euler, self.targetInfo[0:3] - self._coord)
        TargetSpeed = self.TargetSpeed(euler)

        TargetSpeedXZ = np.array([sqrt(pow(TargetSpeed[0], 2) + pow(TargetSpeed[2], 2)),
                                  atan2(-TargetSpeed[2], TargetSpeed[0])])

        sigma_R = -atan2(-TargetCoor[2], TargetCoor[0])
        sigma_T = TargetSpeedXZ[1] - atan2(-TargetCoor[2], TargetCoor[0])
        r = sqrt(pow(TargetCoor[0], 2) + pow(TargetCoor[2], 2))
        d_lambda = (TargetSpeedXZ[0] * sin(sigma_T) - self._speed * sin(sigma_R)) / r
        W = -k_z * self._speed * d_lambda
        n_roll = W / G

        return n_roll

    def M(self):
        return self._speed/300

    def q(self):
        return 0.5*self.p()*self._speed**2

    def p(self):
        return 0.4671

    def D_0(self, V):
        table_coeff = np.array([
            # 250, 300, 320, 500, 900, 1200,
            # 0.0019, 0.0029, 0.0039, 0.0033, 0.0021, 0.0017
            250, 300, 325, 350, 500, 900, 1200,
            0.0037, 0.0057, 0.0075, 0.0075, 0.0062, 0.0041, 0.0031

        ])
        table_coeff = table_coeff.reshape(2, -1).T
        D_0_ = table_coeff[0]
        if V < D_0_[0]:
            return D_0_[1]

        for D_1_ in table_coeff:
            if V <= D_1_[0]:
                return np.arctan((D_1_[1]-D_0_[1])/(D_1_[0]-D_0_[0]))*(V-D_0_[0]) + D_0_[1]
            else:
                D_0_ = D_1_

    def D_a(self, overload):
        C_L = self.m*overload*G/(self.q()*self.s)

        k = 1/np.arctan(0.12/30)
        angle = C_L*k
        D_a = 0.022*np.deg2rad(angle)**2-0.000002*(self._speed-400)

        return D_a

    def CalculateDragForce(self, speed, overload):
        drag = 7*(self.D_0(speed) + self.D_a(overload))

        return drag*self.q()*self.s/(self.m*G)

    def dataForNzPN(self):
        euler = np.copy(self._euler)
        euler[2] = 0
        TargetCoor = toSpeedCoordinateSystem(euler, self.targetInfo[0:3] - self._coord)
        TargetSpeed = self.TargetSpeed(euler)

        TargetSpeedXZ = np.array([sqrt(pow(TargetSpeed[0], 2) + pow(TargetSpeed[2], 2)),
                                  atan2(-TargetSpeed[2], TargetSpeed[0])])

        sigma_R = -atan2(-TargetCoor[2], TargetCoor[0])

        sigma_T = TargetSpeedXZ[1] - atan2(-TargetCoor[2], TargetCoor[0])

        r = sqrt(pow(TargetCoor[0], 2) + pow(TargetCoor[2], 2))

        d_lambda = (TargetSpeedXZ[0] * sin(sigma_T) - self._speed * sin(sigma_R)) / r
        # W = -k_z * self._speed * d_lambda
        # 0 2 3 4 5
        return np.hstack([TargetCoor, TargetSpeed, TargetSpeedXZ, 
                          sigma_R, sigma_T, r, d_lambda, -(self._speed * d_lambda)/G])

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
        self._overload[0] -= self.CalculateDragForce(self._speed, self._overload[1])

        self.integrate(self._d_t)
        self._t += self._d_t 

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
    
    @property
    def angleToTarget(self):
        target_coor = self.targetInfo[0:3]
        v1 = target_coor - self._coord
        euler = self._euler
        # euler[1] = -euler[1]
        # v1[0], v1[1] = v1[1], v1[0]
        v2 = toTrajectoryCoordinateSystem(euler, [1, 0, 0])

        angle = np.arccos(np.dot(v1, v2) /
                          (np.linalg.norm(v1) * np.linalg.norm(v2)))
        
        return angle

    @property
    def speed(self):
        return self._speed

    @property
    def targetLost(self):
        if abs(self.angleToTarget) > self._limit_angle:
            return True
        return False

    @property
    def targetBehind(self):
        if abs(self.angleToTarget) > self._limit_angle+np.deg2rad(10) or self._speed < 200:
            return True
        return False

    @property
    def distanceToTarget(self):
        return np.linalg.norm(self.targetInfo[0:3] - self._coord)

    @property
    def timeExceeded(self):
        if self._t > self._max_time:
            return True
        return False


class LA:
    def __init__(self, coord, euler, speed, d_t, maneuver=0):
        self.start_maneuver_angle = euler[1]
        self.maneuver_sign = 1
        self.triger = True
        self._coord = np.array(coord, dtype=np.float)
        self._speed = speed
        self._euler = np.array(euler, dtype=np.float)
        self._overload = np.array([0, 1, 0], dtype=np.float)
        self._d_t = d_t
        self.t = 0
        self.maneuver_type = maneuver

    @property
    def state(self):
        """ coord, euler, speed, overload """
        return np.hstack([np.copy(self._coord),
                np.copy(self._euler),
                self._speed,
                np.copy(self._overload)])

    def integrate(self, dt):
        self._speed += (self._overload[0] - sin(self._overload[0])) * G * dt
        self._euler[0] += (self._overload[1] * cos(self._euler[2]) - cos(self._euler[0])) * G / self._speed * dt
        self._euler[1] += -self._overload[1] * sin(self._euler[2]) * G / (self._speed * cos(self._euler[0])) * dt
        self._coord[0] += self._speed * cos(self._euler[0]) * cos(self._euler[1]) * dt
        self._coord[1] += self._speed * sin(self._euler[0]) * dt
        self._coord[2] += -self._speed * cos(self._euler[0]) * sin(self._euler[1]) * dt

    def step(self):
        self.grav_compensate()
        # maneuver_map = self.maneuver()

        overload = self.maneuver()
        
        # if isinstance(maneuver_map, np.ndarray):
        #     for index, obj in enumerate(maneuver_map):
        #         if self.t < maneuver_map[0,0]:
        #             break
        #
        #         if maneuver_map[index, 0] < self.t < maneuver_map[index + 1,0]:
        #             overload = obj[1]
        #             break

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

    def maneuver(self):
        if self.maneuver_type == 0:
            start_sec = 5
            end_sec = 2000
            each_sec = 5
            nz = 3
            time_stamps = 3 + np.arange(np.ceil((end_sec-start_sec)/each_sec)+1)*each_sec
            time_stamps[0] += each_sec/2
            overloads = np.tile(np.array([nz,-nz]), time_stamps.shape[0])[:time_stamps.shape[0]]
            maneuver_map = np.array([np.array([time_stamps[i], overloads[i]]) for i in range(time_stamps.shape[0])])

        if self.maneuver_type == 1:
            start_sec = 5
            end_sec = 2000
            each_sec = 10
            nz = 5
            time_stamps = 0 + np.arange(np.ceil((end_sec-start_sec)/each_sec)+1)*each_sec
            time_stamps[0] += each_sec/2
            overloads = np.tile(np.array([nz,-nz]), time_stamps.shape[0])[:time_stamps.shape[0]]
            overloads = overloads
            maneuver_map = np.array([np.array([time_stamps[i], overloads[i]]) for i in range(time_stamps.shape[0])])

        if self.maneuver_type == 2:
            start_sec = 3
            manouver_angle = 45

            if self.triger:
                manouver_angle *= 2/3

            if self.maneuver_sign < 0 and self.start_maneuver_angle+np.deg2rad(manouver_angle) < self._euler[1]:
                self.maneuver_sign = 1
                self.triger = False

            if self.maneuver_sign > 0 and self.start_maneuver_angle-np.deg2rad(manouver_angle) > self._euler[1]:
                self.maneuver_sign = -1
                self.triger = False

            return 3*self.maneuver_sign

        if self.maneuver_type == 3:
            start_sec = 3
            manouver_angle = 45

            if self.triger:
                manouver_angle *= 2/3

            if self.maneuver_sign < 0 and self.start_maneuver_angle+np.deg2rad(manouver_angle) < self._euler[1]:
                self.maneuver_sign = 1
                self.triger = False

            if self.maneuver_sign > 0 and self.start_maneuver_angle-np.deg2rad(manouver_angle) > self._euler[1]:
                self.maneuver_sign = -1
                self.triger = False

            return 4*self.maneuver_sign

        if self.maneuver_type == 4:
            start_sec = 3
            manouver_angle = 45

            if self.triger:
                manouver_angle *= 2/3

            if self.maneuver_sign < 0 and self.start_maneuver_angle+np.deg2rad(manouver_angle) < self._euler[1]:
                self.maneuver_sign = 1
                self.triger = False

            if self.maneuver_sign > 0 and self.start_maneuver_angle-np.deg2rad(manouver_angle) > self._euler[1]:
                self.maneuver_sign = -1
                self.triger = False

            return 5*self.maneuver_sign


        if self.maneuver_type == 0 or self.maneuver_type == 1:
            for index, obj in enumerate(maneuver_map):
                if self.t < maneuver_map[0,0]:
                    return 0

                if maneuver_map[index, 0] < self.t < maneuver_map[index + 1,0]:
                    return obj[1]

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

temp = np.arange(-1,1+0.1,0.1)
# temp = np.array([-7, -5, -4, -3, -2, -1, -0.5, -0.3, -0.2, -0.1, 0, 0.1, 0.2, 0.3, 0.5, 1, 2, 3, 4, 5, 7])
temp = np.hstack([np.arange(-7, -1), np.arange(-1, 1, 0.1), np.arange(1, 8)])
ALL_POSSIBLE_ACTIONS = np.vstack([temp, np.zeros(temp.shape[0])]).T