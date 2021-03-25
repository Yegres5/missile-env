import numpy as np
from math import copysign


def nz_approx(x):
    sign = copysign(1, x)
    if -10 <= x <= 10:
        return 1 / 2 * x
    elif -17 <= x <= 17:
        return (-15 + 2 * abs(x)) * sign
    else:
        return 20 * sign

ran = np.arange(-18,19)
{np.where(ran == x)[0][0]:"Nz = %i" % x for x in ran}