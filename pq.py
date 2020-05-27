#!/usr/bin/env python3
# -*- coding: iso-8859-15 -*-

import numpy as np
from queue import PriorityQueue


class foo:
    def __init__(self, rate):
        self.rate = rate

    def print(self):
        print(self.rate)

    def __lt__(self, obj):
        return self.rate < obj.rate

    def __le__(self, obj):
        return self.rate <= obj.rate

    def __eq__(self, obj):
        return self.rate == obj.rate

    def __ne__(self, obj):
        return self.rate != obj.rate

    def __gt__(self, obj):
        return self.rate > obj.rate

    def __ge__(self, obj):
        return self.rate >= obj.rate


if __name__ == '__main__':
    q = PriorityQueue()
    bars = [foo(np.random.uniform(-1,1)) for _ in range(15)]
    for bar in bars:
        q.put(bar)

    while not q.empty():
        q.get().print()
