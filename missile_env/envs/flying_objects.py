import numpy as np
import matplotlib as mpl
mpl.use('Qt5Agg', warn=False, force=True)

import matplotlib.pyplot as plt
import sys

from math import sin, cos, atan, sqrt, atan2, copysign, acos

mpl.matplotlib_fname()

dT = 0.1


class Rocket:
    def __init__(self, coord, euler, speed):
        self._coord = np.array(coord, dtype=np.float)  # All vectors in form [X, Y, Z]
        self._overload = np.array([0, 1, 0], dtype=np.float)
        self._euler = np.array(euler, dtype=np.float)  # [Theta, Psi, Gamma]
        self._speed = speed
        self._target = None
        self._n_y_max = 20

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
        return list((self._coord, self._euler, self._speed, self._overload))

    def step(self, action):
        self.update(dT)

    def grav_compensate(self):

        self._euler[2] = 0
        compens = np.array([0, 1, 0])
        compens = toSpeedCoordinateSystem(self._euler, compens)

        return compens

    def sumOverloads(self, n_pitch, n_roll):
        self._overload[1] += n_pitch
        # delta = 0 if np.isclose(n_roll, 0) else atan(n_roll / self._overload[1])
        # self._euler = self._euler + np.array([0, 0, delta])
        self._euler[2] += 0 if np.isclose(n_roll, 0) else atan(n_roll / self._overload[1])
        self._overload[1] = sqrt(pow(self._overload[1], 2) + pow(n_roll, 2)) * (1 if self._overload[1] > 0 else -1)

    def TargetSpeed(self):

        target_speed = np.array([self.targetInfo[2] * cos(self.targetInfo[1][0]) * cos(self.targetInfo[1][1]),
                                 self.targetInfo[2] * sin(self.targetInfo[1][0]),
                                 -self.targetInfo[2] * cos(self.targetInfo[1][0]) * sin(self.targetInfo[1][1])])

        target_speed = toSpeedCoordinateSystem(self._euler, target_speed);

        # target_speed = np.array([sqrt(pow(target_speed[0], 2) + pow(target_speed[1], 2) + pow(target_speed[2], 2)),
        #                          acos(sqrt(pow(target_speed[0], 2) + pow(target_speed[2], 2)) /
        #                               sqrt(pow(target_speed[0], 2) +
        #                                    pow(target_speed[1], 2) +
        #                                    pow(target_speed[2], 2))) * copysign(target_speed[1], 1),
        #                          atan2(-target_speed[2], target_speed[0])])

        return target_speed

    def CalculateNyPN(self):

        Ky = 7

        TargetCoor = toSpeedCoordinateSystem(self._euler, self.targetInfo[0] - self._coord)
        TargetSpeed = self.TargetSpeed()

        TargetSpeedXY = np.array([TargetSpeed[0] * cos(TargetSpeed[1]) * cos(TargetSpeed[2]),
                                  TargetSpeed[0] * sin(TargetSpeed[1])])

        TargetSpeedXY = np.array([sqrt(pow(TargetSpeedXY[0], 2) + pow(TargetSpeedXY[1], 2)),
                                  atan2(TargetSpeedXY[1], TargetSpeedXY[0])])

        sigma_R_XY = -atan2(TargetCoor[1], TargetCoor[0])
        sigma_T_XY = TargetSpeedXY[1] - atan2(TargetCoor[1], TargetCoor[0])

        r_XY = sqrt(pow(TargetCoor[0], 2) + pow(TargetCoor[1], 2))

        d_lambda_XY = (TargetSpeedXY[0] * sin(sigma_T_XY) - self._speed * sin(sigma_R_XY)) / r_XY

        W_XY = Ky * self._speed * d_lambda_XY
        n_pitch = W_XY / G

        return n_pitch

    def CalculateNzPN(self):

        Kz = 7

        TargetCoor = toSpeedCoordinateSystem(self._euler, self.targetInfo[0] - self._coord)
        TargetSpeed = self.TargetSpeed()

        TargetSpeed = np.array([sqrt(pow(TargetSpeed[0], 2) + pow(TargetSpeed[1], 2) + pow(TargetSpeed[2], 2)),
                                 acos(sqrt(pow(TargetSpeed[0], 2) + pow(TargetSpeed[2], 2)) /
                                      sqrt(pow(TargetSpeed[0], 2) +
                                           pow(TargetSpeed[1], 2) +
                                           pow(TargetSpeed[2], 2))) * copysign(TargetSpeed[1], 1),
                                 atan2(-TargetSpeed[2], TargetSpeed[0])])

        TargetSpeedXZ = np.array([TargetSpeed[0]*cos(TargetSpeed[1]),
                                             TargetSpeed[2]])

        sigma_R = -atan2(-TargetCoor[2], TargetCoor[0])
        sigma_T = TargetSpeedXZ[1] - atan2(-TargetCoor[2],TargetCoor[0])
        r = sqrt(pow(TargetCoor[0],2) + pow(TargetCoor[2],2))
        d_lambda = (TargetSpeedXZ[0]*sin(sigma_T) - self._speed*sin(sigma_R))/r
        W = -Kz*self._speed*d_lambda
        n_roll = W/G

        return n_roll


    def LimitOverload(self):
        self._overload[1] = (self._n_y_max-1) * (1 if self._overload[1] > 0 else -1) if abs(
            self._overload[1]) > (self._n_y_max - 1) else self._overload[1]

    def update(self, dt):

        self._overload = self.grav_compensate()

        self._overload[0] = 0  # no engine compensate

        self.sumOverloads(self.CalculateNyPN(), self.CalculateNzPN())

        self.LimitOverload()

        self.integrate(dt)

    def integrate(self, dt):
        self._speed += (self._overload[0] - sin(self._overload[0])) * G * dt
        self._euler[0] += (self._overload[1] * cos(self._euler[2]) - cos(self._euler[0])) * G / self._speed * dt
        self._euler[1] += -self._overload[1] * sin(self._euler[2]) * G / (self._speed * cos(self._euler[0])) * dt
        self._coord[0] += self._speed * cos(self._euler[0]) * cos(self._euler[1]) * dt
        self._coord[1] += self._speed * sin(self._euler[0]) * dt
        self._coord[2] += -self._speed * cos(self._euler[0]) * sin(self._euler[1]) * dt

        # print(f"Overload = {self._overload}, Coord = {self._coord}")

    def destroyed(self):
        pass

    def targetLost(self):
        pass


class LA:
    def __init__(self, coord, euler, speed):
        self._coord = np.array(coord, dtype=np.float)
        self._speed = speed
        self._euler = np.array(euler, dtype=np.float)
        self._overload = np.array([0, 1, 0], dtype=np.float)

    @property
    def state(self):
        """ coord, euler, speed, overload """
        return list((self._coord, self._euler, self._speed, self._overload))

    def integrate(self, dt):
        self._speed += (self._overload[0] - sin(self._overload[0])) * G * dt
        self._euler[0] += (self._overload[1] * cos(self._euler[2]) - cos(self._euler[0])) * G / self._speed * dt
        self._euler[1] += -self._overload[1] * sin(self._euler[2]) * G / (self._speed * cos(self._euler[0])) * dt
        self._coord[0] += self._speed * cos(self._euler[0]) * cos(self._euler[1]) * dt
        self._coord[1] += self._speed * sin(self._euler[0]) * dt
        self._coord[2] += -self._speed * cos(self._euler[0]) * sin(self._euler[1]) * dt

    def step(self):
        self.integrate(dT)


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


a = LA(coord=[8000, 8000, 0], speed=200, euler=[0, np.radians(180), 0])
b = Rocket(coord=[0, 0, 0], speed=400, euler=[0, 0, 0])
b.captureTarget(a)

rock_coor = []
target_coor = []
rock_overload = []
target_speed_roc = []

for i in range(5000):
    b.step(1)
    a.step()

    rock_coor.append(np.array(b.state[0]))
    target_coor.append(np.array(a.state[0]))
    rock_overload.append(b.state[3])
    target_speed_roc.append(b.TargetSpeed())

    if len(rock_coor) >= 2:
        if np.sqrt(np.sum((rock_coor[-1] - target_coor[-1])**2)) > np.sqrt(np.sum((rock_coor[-2] - target_coor[-2])**2)):
            break

    # data.append(np.vstack((b.state, a.state)))
    # print(f"Rocket = {b.state}, Target = {a.state}")

rock_coor = np.array(rock_coor)
target_coor = np.array(target_coor)

np.savetxt("0.csv", rock_coor[:, [2, 0, 1]], delimiter=",")
np.savetxt("1.csv", target_coor[:, [2, 0, 1]], delimiter=",")

# runfile('/Users/evgeny/Documents/Инст/Мага/Diploma/missile-env/missile_env/envs/3Ddraw_animate.py', wdir='/Users/evgeny/Documents/Инст/Мага/Diploma/missile-env/missile_env/envs')
# exec(open('/Users/evgeny/Documents/Инст/Мага/Diploma/missile-env/missile_env/envs/3Ddraw_animate.py').read())

# rock_overload = np.array(rock_overload).T[1]

plt.figure(figsize=[16, 9])

plt.subplot(221)
tempRock = rock_coor.T[0:2]
tempTarget = target_coor.T[0:2]
plt.plot(tempRock[0], tempRock[1])
plt.plot(tempTarget[0], tempTarget[1])

plt.subplot(222)
tempRock = rock_coor.T[[0, 2]]
tempTarget = target_coor.T[[0, 2]]
plt.plot(tempRock[0], tempRock[1])
plt.plot(tempTarget[0], tempTarget[1])

plt.subplot(223)
plt.plot(rock_overload)

plt.subplot(224)
plt.plot(target_speed_roc)

# plt.subplot(2, 1)
# plt.title("Trace")
# plt.plot(rock_coor.T[0:2])
# plt.plot(target_coor.T[0:2])
# plt.grid()
#
# plt.subplot(2, 2)
# plt.title("Distance")
# plt.plot(np.sqrt(np.sum(((target_coor - rock_coor) ** 2), axis=1)))
# plt.grid()

plt.show(block=True)

sys.exit()