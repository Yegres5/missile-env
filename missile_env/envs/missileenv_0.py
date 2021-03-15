import gym
from gym import error, spaces, utils
from gym.utils import seeding


class MissileEnv_0(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):
        print("init")

    def step(self, action):
        print("step with action = ", action)

    def reset(self):
        print("reset")

    def render(self, mode='human'):
        print('render with mode = ', mode)

    def close(self):
        print('close')
