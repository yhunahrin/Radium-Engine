#! /usr/bin/python

import fileinput
import numpy as np

dict={}

for line in fileinput.input():

    spl = line.split(':')


    if line[0:5]=="frame":
        name = "frame"
    else:
        name = spl[0].split('(')[0]

    if "{" in spl[0] or "}" in spl[0] or spl[0] == "\n":
        continue

    if name not in dict:
        dict[name] = []

    arr = spl[1].split(' ')
    if name=="frame":
        dict[name].append(int(arr[1]))
    else:
        dict[name].append( int(arr[2]))


for x in dict:
    print x
    print np.mean(dict[x])





