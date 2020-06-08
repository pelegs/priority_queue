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

def tripleroduct(v1, v2, v3):
    """
    Return a special triple product
    of the vectors v1, v2 and v3.
    """
    return np.sum([a*b*c for a, b, c in zip(v1, v2, v3)])


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
    """
    vn = np.dot(p.vel, w.normal)
    if vn == 0:
        return inf
    else:
        t1 = (+p.radius - np.dot(p.pos-w.center, w.normal)) / vn
        t2 = (-p.radius - np.dot(p.pos-w.center, w.normal)) / vn
        t = min(t1, t2)
        
        # contact point on plane
        x_s = p.pos + p.vel*(t + p.radius/np.linalg.norm(p.vel))

        # projections of contact point
        # on each directions of plane
        wd1_norm = np.linalg.norm(w.d1)
        wd2_norm = np.linalg.norm(w.d2)
        x_s_d1 = abs(np.dot(x_s, w.d1) / wd1_norm)
        if wd2_norm == 0:
            x_s_d2 = 0.0
        else:
            x_s_d2 = abs(np.dot(x_s, w.d2) / wd2_norm)

        
        # verifying that the contact point is
        # within the wall
        if x_s_d1 <= wd1_norm and\
           x_s_d2 <= wd2_norm:
            with open('temp.txt', 'a') as f:
                f.write('{}, {}\n'.format(x_s_d1, np.linalg.norm(w.d1)))
            return t
        else:
            with open('temp.txt', 'a') as f:
                f.write('{}, {}\n'.format(x_s_d1, np.linalg.norm(w.d1)))
            return inf

def pw_collision(p, w):
    """
    Return the new particle velocity
    after a collision with a wall.
    """
    return p.vel - 2*np.dot(p.vel, w.normal)*w.normal


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
    def __init__(self, center, d1, d2=None):
        self.center = center
        self.d1 = d1

        # Set surface normal.
        # If 3D, by cross product.
        # If 2D, set d2 to (0,0,1), do as 3D,
        # and truncate to 2D.
        if d1.shape[0] == 3:
            self.d2 = d2
            self.normal = -normalize(np.cross(d1, self.d2))
            self.vertices = [
                    self.center + self.d1 + self.d2,
                    self.center + self.d1 - self.d2,
                    self.center - self.d1 - self.d2,
                    self.center - self.d1 + self.d2
                    ]
                    
        elif d1.shape[0] == 2:
            D1 = np.append(d1, 0)
            D2 = np.array([0,0,1])
            self.normal = -normalize(np.cross(D1, D2))[:2]
            self.d2 = np.zeros(2)

    def tikz_draw(self, fill='col3!30', draw_dirvecs=False):
        drawstr = """
        \\draw[wall, fill={}] ({},{},{}) -- ({},{},{}) -- ({},{},{}) -- ({},{},{}) -- cycle;
              """.format(
                      fill,
                      self.vertices[0][0], self.vertices[0][1], self.vertices[0][2],
                      self.vertices[1][0], self.vertices[1][1], self.vertices[1][2],
                      self.vertices[2][0], self.vertices[2][1], self.vertices[2][2],
                      self.vertices[3][0], self.vertices[3][1], self.vertices[3][2],
                      )
        if draw_dirvecs:
            drawstr += """
            \\draw[vector, ultra thick, col1] ({},{},{}) -- ++({},{},{});
            \\draw[vector, ultra thick, col2] ({},{},{}) -- ++({},{},{});
            """.format(
                    self.center[0], self.center[1], self.center[2],
                    self.d1[0], self.d1[1], self.d1[2],
                    self.center[0], self.center[1], self.center[2],
                    self.d2[0], self.d2[1], self.d2[2],
                    )
        print(drawstr)


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
    while t == inf or t < 1 or t > 20:
        p = Particle(
                pos = np.zeros(2),
                vel = np.random.uniform(-2, 2, 2),
                radius = np.random.uniform(0.2, 2),
                )
        w = Wall(
                center = np.random.uniform(-100,100,2),
                d1 = np.random.uniform(-1,1,2)
                )
        t = time_to_pw_collision(p, w)
    print(t)
