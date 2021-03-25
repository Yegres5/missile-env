import gym
import numpy as np

from wrapper import Wrapper, ACTION_MEANING
from gym import spaces
from gym.utils import seeding


def to_ram(wrap):
    ram_size = wrap.getRAMSize()
    ram = np.zeros(ram_size, dtype=np.uint8)
    wrap.getRAM(ram)
    return ram


class MissileEnv0(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self, obs_type='ram', frameskip=1):
        print("init")

        self._obs_type = obs_type
        self.frameskip = frameskip
        self.wrap = Wrapper()

        self.seed()
        self._action_set = self.wrap.getLegalActionSet()
        self.action_space = spaces.Discrete(len(self._action_set))

    def seed(self, seed=None):
        self.np_random, seed1 = seeding.np_random(seed)
        seed2 = seeding.hash_seed(seed1 + 1) % 2**31
        self.wrap.setInt(b'random_seed', seed2)

        return [seed1, seed2]

    def step(self, action):
        print("step with action = ", action)
        reward = 0.0
        action = self._action_set[action]

        if isinstance(self.frameskip, int):
            num_steps = self.frameskip
        else:
            assert isinstance(self.frameskip, list)
            num_steps = self.np_random.randint(self.frameskip[0], self.frameskip[1])
        for _ in range(num_steps):
            reward += self.wrap.act(action)
        ob = self._get_obs

        return ob, reward, self.wrap.game_over, ""

    @property
    def _get_obs(self):
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

    def reset(self):
        print("reset")

    def render(self, mode='human'):
        print('render with mode = ', mode)

    def close(self):
        print('close')
