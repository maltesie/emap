from itertools import tee

def pairwise(iterable):
    "s -> (s0,s1), (s1,s2), (s2, s3), ..."
    a, b = tee(iterable)
    next(b, None)
    return zip(a, b)

def remove_multiple(my_list, indexes):
    for index in sorted(indexes, reverse=True):
        del my_list[index]