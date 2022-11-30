import numpy as np
from missile_env.envs.flying_objects import Rocket, LA, ALL_POSSIBLE_ACTIONS
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
        self.start_path = 0

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

    def numberToOverloads(self, num_action):
        return ALL_POSSIBLE_ACTIONS[num_action]

    @property
    def rocketSpeed(self):
        return self.rocket.speed

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
            values_min = min_indexes[np.argmax(n2_legal[min_indexes]-n2_overload)]
        else:
            # values_min = np.argmin(n2_legal[values_min])
            values_min = values_min[np.argmin(n2_legal[values_min])]

        # final_index = values_min[np.argmin(n2_legal[values_min])]
        
        # final_index = min_indexes[values_min]#min_indexes[np.argmin(abs(n2_legal[min_indexes] - n2_overload))]

        return overload_list[values_min]


    def setInt(self, param, seed2):
        """ set seeds? """
        np.random.seed(seed2)

    @property
    def state(self):
        return self.rocket.state + self.target.state + [self.start_path]

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


        # reward += -0.05
        
        # if self.rocket.destroyed:
        #     reward += 5
        # elif self.rocket.targetLost():
        #     reward += -10

        if self.rocket.targetLost:
            reward -= 0#0.05
        else:
            reward -= 0#0.01


        # reward -= 0.01*np.exp(1/10000*(self.rocket.distanceToTarget - 10000))

        # reward +=  0.01*np.exp(-1/5000*self.rocket.distanceToTarget)

        if self.game_over:
            reward += 0#10*np.exp(-1/5000*self.rocket.distanceToTarget) + 10*np.exp(-1/300*self.rocket.distanceToTarget)

            if not self.rocket.destroyed:
                reward -= 0
                # reward -= -1/1000*self.rocket.distanceToTarget
                reward += -20/18000*self.rocket.distanceToTarget

            else:
                reward += 1#4
            # reward += -self.rocket.distanceToTarget/(18000/10)+10

        # if self.rocket.distanceToTarget > 18000:
        #     reward -= 0.5
        # elif self.rocket.distanceToTarget > 10000:
        #     reward -= 0.4
        # elif self.rocket.distanceToTarget > 5000:
        #     reward -= 0.3
        # elif self.rocket.distanceToTarget > 2000:
        #     reward -= 0.2
        
        
        return reward

    @property
    def game_over(self):
        if self.rocket.destroyed or self.rocket.timeExceeded or self.rocket.targetBehind:
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
        # self.start_path = np.sign(info["t_euler"][1])
        
        r_coor, r_speed, r_euler = copy.deepcopy(self.ini_rocket_info)
        t_coor, t_speed, t_euler = copy.deepcopy(self.ini_target_info)
        maneuver = 0
        
        # Not working git
        #Checking
        # info = {"r_euler": [0, np.random.uniform(np.deg2rad(-5), np.deg2rad(5)), 0],
        #         "t_euler": [0, np.random.uniform(np.deg2rad(-90), np.deg2rad(90)), 0]}

        if "la_coord" in info:
            t_coor = info["la_coord"]

        if "r_euler" in info:
            r_euler = [r_euler[i] + value for i,value in enumerate(info["r_euler"]) ]

        if "t_euler" in info:
            t_euler = [t_euler[i] + value for i,value in enumerate(info["t_euler"]) ]

        if "maneuver" in info:
            maneuver = info["maneuver"]

        self.start_path = np.sign(t_euler[1])

        self.rocket = Rocket(coord=r_coor, euler=r_euler, speed=r_speed, d_t=self.d_t)
        self.target = LA(coord=t_coor, euler=t_euler, speed=t_speed, d_t=self.d_t, maneuver=maneuver)
        self.rocket.captureTarget(self.target)
        return self.state


