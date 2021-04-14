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


a = LA(coord=[4000, 4000, 2000], speed=300, euler=[0, np.radians(90), 0], d_t=0.1)
b = Rocket(coord=[0, 0, 0], speed=400, euler=[0, 0, 0], d_t=0.1)
b.captureTarget(a)

rock_coor = []
target_coor = []
rock_overload = []
target_speed_roc = []

for i in range(1000):
    b.step(1)
    a.step()

    rock_coor.append(np.array(b.state[0]))
    target_coor.append(np.array(a.state[0]))
    rock_overload.append(b.state[3])
    target_speed_roc.append(b.TargetSpeed())

    if len(rock_coor) >= 2:
        if np.sqrt(np.sum((rock_coor[-1] - target_coor[-1]) ** 2)) > np.sqrt(
                np.sum((rock_coor[-2] - target_coor[-2]) ** 2)):
            break

rock_coor = np.array(rock_coor)
target_coor = np.array(target_coor)

np.savetxt("0.csv", rock_coor[:, [2, 0, 1]], delimiter=",")
np.savetxt("1.csv", target_coor[:, [2, 0, 1]], delimiter=",")

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

plt.show(block=True)

sys.exit()
