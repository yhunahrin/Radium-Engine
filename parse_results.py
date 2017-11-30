#! /usr/bin/python

import fileinput
import numpy as np
import operator

dict={}

for line in fileinput.input():

    spl = line.split(':')


    if line[0:5]=="frame":
        name = "frame"
    else:
        name = spl[0].split('(')[0]
        name = name.strip()

    if "{" in spl[0] or "}" in spl[0] or spl[0] == "\n":
        continue

    if name not in dict:
        dict[name] = []

    arr = spl[1].split(' ')
    if name=="frame":
        dict[name].append(int(arr[1]))
    else:
        dict[name].append( int(arr[-1]))


averages=[]
for x in dict:
    averages.append( (x, np.mean(dict[x])))

averages.sort(key=operator.itemgetter(1))
averages.reverse()
for a in averages:
    print a








