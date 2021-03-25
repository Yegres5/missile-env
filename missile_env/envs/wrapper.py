import numpy as np

from flying_objects import Rocket, LA
from math import copysign


class Wrapper:
    def __init__(self):
        self._nz_gap = 5
        self._rocket = Rocket()
        self._target = LA()

        self._state = np.hstack(self._target.state(), self._rocket.state)

    def getLegalActionSet(self):
        nz = self._rocket.state
        legal_nz = ALL_NZ[abs(ALL_NZ) - nz <= self._nz_gap]

        return legal_nz

    def setInt(self, param, seed2):
        pass

    def act(self, action):
        self._target.step()
        self._rocket.step(action)

        self._state = np.hstack(self._target.state(), self._rocket.state)

        reward = 0
        return reward

    @property
    def game_over(self):
        if self._rocket.destroyed() or self._rocket.targetLost():
            return True
        return False

    def getScreen(self):
        pass

    def getRAMSize(self):
        pass

    def getRAM(self):
        """Current observation. Return np.array(), not image. Why RAM?"""

        return self._state


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
