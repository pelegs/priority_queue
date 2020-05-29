#!/usr/bin/env python3
# -*- coding: iso-8859-15 -*-

import numpy as np
from queue import PriorityQueue


#############
# Constants #
#############

DIM = 2


####################
# Helper functions #
####################

def time_to_pp_collision(p1, p2):
    """
    Returns time to collision between
    two particles p1, p2.
    """
    dx02 = np.linalg.norm(p1.pos - p2.pos)**2
    dv2 = np.linalg.norm(p1.v - p2.v)**2
    R2 = (p1.radius + p2.radius)**2
    desc = np.dot(dx0, dv)**2 - dv2*(dx02 - R2)
    if desc >= 0 and dv2 != 0:
        p = (np.dot(dx0, dv) + np.sqrt(desc))/dv2
        m = (np.dot(dx0, dv) - np.sqrt(desc))/dv2
        return min(p, m)
    else:
        return None


###########
# Classes #
###########

class Particle:
    def __init__(self, pos, vel, mass=1, radius=1):
        self.pos = pos
        self.vel = vel
        self.mass = mass
        self.radius = radius


def pp_collision(p1, p2):
    x1, x2 = p1.pos, p2.pos
    dx = x1 - x2
    v1, v2 = p1.vel, p2.vel
    dv = v1 - v2
    A = np.dot(dv, dx) / np.linalg.norm(dx)**2

    m1, m2 = p1.mass, p2.mass
    M = m1 + m2

    u1 = v1 - 2*m2/M * (+dx) * A
    u2 = v2 - 2*m1/M * (-dx) * A

    return u1, u2

class Event:
    def __init__(self, time):
        self.time = time
        self.valid = True

    def invalidate(self):
        self.valid = False

    def print(self):
        print(self.time)

    def __lt__(self, event):
        return self.time < event.time

    def __le__(self, event):
        return self.time <= event.time

    def __eq__(self, event):
        return self.time == event.time

    def __ne__(self, event):
        return self.time != event.time

    def __gt__(self, event):
        return self.time > event.time

    def __ge__(self, event):
        return self.time >= event.time


if __name__ == '__main__':
    q = PriorityQueue()
    bars = [foo(np.random.uniform(-1,1)) for _ in range(15)]
    for bar in bars:
        q.put(bar)

    while not q.empty():
        q.get().print()
