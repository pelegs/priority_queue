#!/usr/bin/env python3
# -*- coding: iso-8859-15 -*-

import numpy as np
from math import inf
from queue import PriorityQueue


#############
# Constants #
#############


##################
# Math functions #
##################

def normalize(vec):
    try:
        return vec / np.linalg.norm(vec)
    except:
        return 0.0


####################
# Helper functions #
####################

def time_to_pp_collision(p1, p2):
    """
    Returns time to collision between
    two particles p1, p2.
    """
    dx0 = p1.pos - p2.pos
    dx02 = np.dot(dx0, dx0)
    dv = p2.vel - p1.vel
    dv2 = np.dot(dv, dv)
    R2 = (p1.radius + p2.radius)**2
    desc = np.dot(dx0, dv)**2 - dv2*(dx02 - R2)

    if desc >= 0 and dv2 != 0:
        p = (np.dot(dx0, dv) + np.sqrt(desc)) / dv2
        m = (np.dot(dx0, dv) - np.sqrt(desc)) / dv2
        return min(p, m)
    else:
        return inf

def pp_collision(p1, p2):
    """
    Returns the new velocities after
    a particle-particle collision.
    """
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


def time_to_pw_collision(p, w):
    """
    Returns the time at which the particle
    p collides with the wall w.
    NOTE: NOT CORRECT! SHOULD BE RE-WRITTEN.
    """
    p0 = w.center
    n = w.normal
    l = p.vel
    l0 = p.pos
    l_n = np.dot(l, n)
    if l_n == 0:
        return inf
    else:
        # Time to collision
        t = ((p0-l0)*n) / np.dot(l, n)

        # Position of collision
        d = l0 + l*t

        # Verifiying that d is inside the wall
        # by projecting the vector from d to
        # the center of the wall on the wall's
        # direction vectors.
        D = d - p0
        a = np.dot(D, normalize(w.d1))
        b = np.dot(D, normalize(w.d2))

        # Return t if D is inside the wall, inf otherwise.
        if a <= np.linalg.norm(w.d1) and b <= np.linalg.norm(w.d2):
            return t
        else:
            return inf

def pw_collision(p, w):
    """
    Return the new particle velocity
    after a collision with a wall.
    """
    return p.vel - 2*np.dot(p.vel, w.normal)*w.normal

def bla(d, n):
    return d - 2*np.dot(d, n)*n


###########
# Classes #
###########

class Particle:
    def __init__(self, pos, vel, mass=1, radius=1):
        self.pos = pos
        self.vel = vel
        self.mass = mass
        self.radius = radius

    def move(self, dt):
        self.pos += self.vel * dt

    def print(self):
        print('x=({}), v=({}), m={:0.2f}, R={:0.2f}'.format(
            ','.join(['{:0.2f}'.format(x) for x in self.pos]),
            ','.join(['{:0.2f}'.format(x) for x in self.vel]),
            self.mass, self.radius)
            )


class Wall:
    def __init__(self, center, d1, d2):
        self.center = center
        self.d1 = d1
        self.d2 = d2
        self.normal = normalize(np.cross(d1, d2))


class Event:
    def __init__(self, time, obj1, obj2):
        self.time = time
        self.obj1 = obj1
        self.obj2 = obj2
        self.valid = True

    def invalidate(self):
        self.valid = False

    def print(self):
        print(self.time)

    """
    Comparing between events is done
    via their time.
    """
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


########
# Main #
########

if __name__ == '__main__':
    """
    q = PriorityQueue()
    bars = [foo(np.random.uniform(-1,1)) for _ in range(15)]
    for bar in bars:
        q.put(bar)

    while not q.empty():
        q.get().print()
    """

    t = inf
    while t == inf or t < 0.5 or t > 10:
        p1 = Particle(
                pos = np.random.uniform(-10, 10, 2),
                vel = np.random.uniform(-4, 4, 2),
                mass = np.random.uniform(1, 5),
                radius = np.random.uniform(0.25, 1.5),
                )
        p2 = Particle(
                pos = np.random.uniform(-10, 10, 2),
                vel = np.random.uniform(-4, 4, 2),
                mass = np.random.uniform(1, 5),
                radius = np.random.uniform(0.25, 1.5),
                )

        t = time_to_pp_collision(p1, p2)