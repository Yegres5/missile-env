import numpy as np
from missile_env.envs.flying_objects import Rocket, LA, ALL_NZ, ALL_POSSIBLE_ACTIONS
import sys
import matplotlib.pyplot as plt
import copy

class Wrapper:
    rocket = None
    target = None
    d_t = 0.1
    ny_gap = 20

    def __init__(self, rocket_info, target_info):
        self.ini_rocket_info = rocket_info
        self.ini_target_info = target_info

        self.reset()

        self.distance_to_target = self.rocket.distanceToTarget

        self.reward_program = np.array([[15000, 100], [10000, 100], [5000, 100], [2000, 100], [500, 100]])

    def getFullActionSet(self):
        return ALL_POSSIBLE_ACTIONS

    def getLegalOverloads(self):
        # FIXME: current overloads change 
        ny = self.rocket._current_overloads[0] # Module of Ny
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
        overload = np.round(overload, 4)

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
        min_indexes = np.where(np.less_equal(diff, value))[0]

        values_min = min_indexes[n2_legal[min_indexes]-n2_overload > 0]
        
        if not values_min.size:
            values_min = np.argmax(n2_legal[min_indexes]-n2_overload)
        else:
            values_min = np.argmin(n2_legal[values_min])

        # final_index = values_min[np.argmin(n2_legal[values_min])]
        
        # final_index = min_indexes[values_min]#min_indexes[np.argmin(abs(n2_legal[min_indexes] - n2_overload))]

        return overload_list[min_indexes[values_min]]


    def setInt(self, param, seed2):
        """ set seeds? """
        pass

    @property
    def state(self):
        return np.hstack((np.array(self.rocket.state, dtype=object), np.array(self.target.state, dtype=object)))

    def act(self, action):
        """ action: np.array 1x2 [Nz, Ny] """
        self.target.step()
        self.rocket.grav_compensate()
        # print("Overload that should be is = ", self.rocket.proportionalCoefficients(k_z=2, k_y=2))
        self.rocket.step(action)  # FIXME: quick fix

        self.distance_to_target = self.rocket.distanceToTarget

        reward = 0
        # if self.reward_program.size:
        #     if self.reward_program[0,0] > self.distance_to_target:
        #         reward += self.reward_program[0, 1]
        #         self.reward_program = np.delete(self.reward_program, 0, axis=0)


        reward += -0.1
        if self.rocket.destroyed:
            reward += 10
        elif self.rocket.targetLost():
            reward = -40


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
        self.reward_program = np.array([[15000, 100], [10000, 100], [5000, 100], [2000, 100], [500, 100]])
        
        r_coor, r_speed, r_euler = copy.deepcopy(self.ini_rocket_info)
        t_coor, t_speed, t_euler = copy.deepcopy(self.ini_target_info)
        
        if "la_coord" in info:
            info["la_coord"][1] = 0
            t_coor = t_coor + info["la_coord"]

        if "r_euler" in info:
            r_euler += info["r_euler"]

        if "t_euler" in info:
            t_euler += info["t_euler"]


        self.rocket = Rocket(coord=r_coor, euler=r_euler, speed=r_speed, d_t=self.d_t)
        self.target = LA(coord=t_coor, euler=t_euler, speed=t_speed, d_t=self.d_t)
        self.rocket.captureTarget(self.target)
        return self.state