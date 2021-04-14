import numpy as np
from missile_env.envs.flying_objects import Rocket, LA, ALL_NZ, ALL_POSSIBLE_ACTIONS
import sys
import matplotlib.pyplot as plt


class Wrapper:
    rocket = None
    target = None
    d_t = 0.1
    ny_gap = 5

    def __init__(self, rocket_info, target_info):
        self.ini_rocket_info = rocket_info
        self.ini_target_info = target_info

        self.reset()

        self.distance_to_target = self.rocket.distanceToTarget

    def getFullActionSet(self):
        return ALL_POSSIBLE_ACTIONS

    def getLegalOverloads(self):
        ny = self.rocket.state[3][1] # Module of Ny
        diff = np.sqrt(np.sum(ALL_POSSIBLE_ACTIONS ** 2, axis=1)) - ny
        legal_n = ALL_POSSIBLE_ACTIONS[np.ix_((diff <= self.ny_gap)), :].reshape(-1, 2)

        return legal_n

    def getLegalActionSet(self):
        return self.overloadsToNumber(self.getLegalOverloads())

    def overloadsToNumber(self, overloads):
        _, indexes = np.where([np.all(ALL_POSSIBLE_ACTIONS == i, axis=1) for i in overloads])
        return indexes

    def findClosestFromLegal(self, overload):
        # FIXME:wrong algorithm [-12, 0] -> [-5, 3]
        overload_list = self.getLegalOverloads()

        n2_legal = np.sqrt(np.sum(overload_list ** 2, axis=1))
        n2_overload = np.sqrt(np.sum(overload ** 2))

        angles_legal = np.arctan2(overload_list[:,1], overload_list[:,0])

        angle_overload = np.arctan2(overload[1], overload[0])

        if angle_overload < 0:
            angle_overload += 2*np.pi

        if np.sum(angles_legal < 0):
            angles_legal[angles_legal < 0] += 2*np.pi

        diff = abs(angle_overload - angles_legal)
        min_angle = np.min(diff)

        sorted = np.sort(diff)
        p = 5
        r = (p/100)*(sorted.shape[0] - 1) + 1

        value = sorted[int(np.ceil(r))]

        # min_indexes = np.where(np.equal(diff, min_angle))[0]
        min_indexes = np.where(np.less(diff, value))[0]

        final_index = min_indexes[np.argmin(abs(n2_legal[min_indexes] - n2_overload))]

        return overload_list[final_index]


    def setInt(self, param, seed2):
        """ set seeds? """
        pass

    @property
    def state(self):
        return np.hstack((np.array(self.rocket.state, dtype=object), np.array(self.target.state, dtype=object)))

    def act(self, action):
        """ action: np.array 1x2 [Nz, Ny] """
        self.target.step()
        self.rocket.step(action)

        reward = self.distance_to_target - self.rocket.distanceToTarget
        self.distance_to_target = self.rocket.distanceToTarget

        reward = self.d_t
        return reward

    @property
    def game_over(self):
        if self.rocket.destroyed or self.rocket.targetLost():
            return True
        return False

    def getScreen(self):
        pass

    def getRAMSize(self):
        pass

    def getRAM(self):
        """Current observation. Return np.array(), not image. Why RAM?"""
        return self.state

    def reset(self, **info):
        """  """
        r_coor, r_speed, r_euler = self.ini_rocket_info
        t_coor, t_speed, t_euler = self.ini_target_info
        
        if "la_coord" in info:
            t_coor = t_coor + info["la_coord"]

        self.rocket = Rocket(coord=r_coor, euler=r_euler, speed=r_speed, d_t=self.d_t)
        self.target = LA(coord=t_coor, euler=t_euler, speed=t_speed, d_t=self.d_t)
        self.rocket.captureTarget(self.target)
        return self.state