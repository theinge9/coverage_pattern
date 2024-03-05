#! /usr/bin/env python3

import itertools

## From https://docs.python.org/2.7/library/itertools.html#recipes
def pairwise(iterable):
    "s -> (s0,s1), (s1,s2), (s2,s3), ..."
    a, b = itertools.tee(iterable)
    next(b, None)
    return zip(a, b)

def list_avg_dist(l):
    """Returns the average distance between values in given list"""
    dists = [abs(combo[0]-combo[1]) for combo in pairwise(l)]
    return sum(dists)/len(dists)