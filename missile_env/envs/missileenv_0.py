import gym
import numpy as np
from missile_env.envs.wrapper import Wrapper
from gym import spaces
from gym.utils import seeding
from math import atan
import matplotlib.pyplot as plt
import sys
import yappi

def to_ram(wrap):
    # ram_size = wrap.getRAMSize()
    # ram = np.zeros(ram_size, dtype=object)
    # wrap.getRAM(ram)
    return wrap.getRAM()

class MissileEnv0(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self,
                 rocket_info=[[0, 0, 0], 900, [0, 0, 0]],
                 target_info=[[25000, 0, 0], 200, [0, 0, 0]], obs_type='ram', frameskip=1):

        self._obs_type = obs_type
        self.frameskip = frameskip

        self.wrap = Wrapper(rocket_info=rocket_info, target_info=target_info)

        # self.frameskip = int(np.floor(1/self.wrap.d_t)) # FIXME: action each second

        self.seed(42)
        self._action_set = self.wrap.getFullActionSet()
        self.action_space = spaces.Discrete(len(self._action_set))

    def seed(self, seed=None):
        self.np_random, seed1 = seeding.np_random(seed)
        seed2 = seeding.hash_seed(seed1 + 1) % 2**31
        self.wrap.setInt(b'random_seed', seed2)

        return [seed1, seed2]

    def available_actions(self):
        possible_actions = wrap.getLegalActionSet()
        [np.argwhere(a == j)[0][0] for j in np.array([1,2,3])]

        return wrap.getLegalActionSet()
    
    def step(self, action):
        """ Input action: int """

        reward = 0.0
        action = self._action_set[action]
        gameover = False
        target_hitted = False

        if isinstance(self.frameskip, int):
            num_steps = self.frameskip
        else:
            assert isinstance(self.frameskip, list)
            num_steps = self.np_random.randint(self.frameskip[0], self.frameskip[1])
        for _ in range(num_steps):
            reward += self.wrap.act(action)
            if self.wrap.game_over:
                gameover = True
                # reward += 30*np.exp(-1/8000*self.wrap.rocket.distanceToTarget)
                # reward += -self.wrap.rocket.distanceToTarget*1/(18000/20)+20 #100*np.exp(-1/4000*self.wrap.rocket.distanceToTarget)
                break

        ob = np.array(self.get_obs)
        
        if self.wrap.rocket.destroyed:
            target_hitted = True

        return ob, reward, self.wrap.game_over, {"Destroyed": target_hitted, 
                                                 "Distance": self.wrap.distance_to_target,
                                                 "Final speed": self.wrap.rocketSpeed}

    @property
    def get_obs(self):
        if self._obs_type == 'ram':
            return self._get_ram()
        elif self._obs_type == 'image':
            img = self._get_image()
            return img
        assert True, "Wrong _obs_type"

    def _get_image(self):
        return self.wrap.getScreen()

    def _get_ram(self):
        return to_ram(self.wrap)

    def reset(self, **info):
        return self.wrap.reset(**info)

    def render(self, mode='human'):
        print('render with mode = ', mode)

    def close(self):
        print('close')

    def actionsForK(self, coefficients):
        over = self.wrap.rocket.proportionalCoefficients(coefficients[0],
                                                         coefficients[1])


# m = MissileEnv0()
#
# log = []
# reward = 0
#
# log.append(np.copy(m.wrap.state))
# true_overload = []
#
# for _ in range(3000):
#     m.wrap.rocket.grav_compensate()
#     overload = m.wrap.rocket.proportionalCoefficients(k_z=2, k_y=2)
#     true_overload.append(overload)
#
#     possible = m.wrap.findClosestFromLegal(overload)
#     action_num = m.wrap.overloadsToNumber([possible])
#
#     ob, r, done, info = m.step(action_num[0])
#
#     reward += r
#
#     log.append(np.copy(ob))
#     print(m.wrap.distance_to_target)
#     if done:
#         break
#
#
# print("Reward = ", reward)
# log = np.array(log)
#
# rocket_log = [item[0] for item in log]
# la_log = [item[1] for item in log]
#
# rocket_coord = np.vstack([item[0] for item in rocket_log])
# la_coord = np.vstack([item[0] for item in la_log])
# np.savetxt("0.csv", rocket_coord[:,[2,0,1]], delimiter=",")
# np.savetxt("1.csv", la_coord[:,[2,0,1]], delimiter=",")
#
# overloads = np.copy(np.vstack([item[4] for item in rocket_log]))
#
# plt.figure(figsize=[16, 9])
#
# ax1 = plt.subplot(221)
# ax1.plot(rocket_coord.T[0], rocket_coord.T[1])
# ax1.plot(la_coord.T[0], la_coord.T[1])
# ax1.set_xlabel("X")
# ax1.set_ylabel("Y")
#
# ax2 = plt.subplot(222)
# ax2.plot(rocket_coord.T[0], rocket_coord.T[2])
# ax2.plot(la_coord.T[0], la_coord.T[2])
# ax2.set_xlabel("X")
# ax2.set_ylabel("Z")
#
# ax3 = plt.subplot(223)
# ax3.plot(rocket_coord.T[2], rocket_coord.T[1])
# ax3.plot(la_coord.T[2], la_coord.T[1])
# ax3.set_xlabel("Z")
# ax3.set_ylabel("Y")
#
# true_overload = np.array(true_overload)
# print(true_overload)
#
# ax4 = plt.subplot(224)
# ax4.plot(overloads.T[0], label="Nz", color="green")
# ax4.plot(overloads.T[1], label="Ny", color="blue")
# ax4.plot(true_overload.T[0], label="True Nz", linestyle='dashed', color="green")
# ax4.plot(true_overload.T[1], label="True Ny", linestyle='dashed', color="blue")
# ax4.set_xlabel("Iteration")
# ax4.set_ylabel("Overload")
# ax4.legend()
#
# plt.show(block=True)
#
# sys.exit()
