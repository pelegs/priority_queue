#!/usr/bin/env python3
# -*- coding: iso-8859-15 -*-

import numpy as np


def collision(dx0, dv, R):
    dx02 = np.linalg.norm(dx0)
    dv2 = np.linalg.norm(dv)**2
    dx02 = np.linalg.norm(dx0)**2
    desc = np.dot(dx0,dv)**2 - dv2*(dx02-R**2)
    if desc >= 0 and dv2 != 0:
        p = (np.dot(dx0,dv) + np.sqrt(desc))/dv2
        m = (np.dot(dx0,dv) - np.sqrt(desc))/dv2
        if p == m:
            print('t =', p)
        else:
            print('t12 = {:0.2f}, {:0.2f}'.format(p, m))
    else:
        print('No collision')


if __name__ == '__main__':
    x10 = np.array([0,0])
    x20 = np.array([5,0])
    v1 = np.array([-1,0])
    v2 = np.array([-2,0.1])
    r1 = 0.5
    r2 = 0.5
    collision(x10-x20, v2-v1, r1+r2)
