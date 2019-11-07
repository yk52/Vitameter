#!/usr/bin/env python
import numpy as np
import csv

# ask for input through terminal
filename = input("Enter filename: ")


f = open(filename)
for line in f:
    if (':' in line):
        row = line.split(':')
        if ('CO2' in row[0]):
            co2 = row[1].split(',')
            co2 = co2[:-1]
            i = 0
            for x in co2:
                co2[i] = int(x)
                i += 1
        if ('VOC' in row[0]):
            voc = row[1].split(',')
            voc = voc[:-1]
            i = 0
            for x in voc:
                voc[i] = int(x)
                i += 1
        if ('UVI' in row[0]):
            uvi = row[1].split(',')
            uvi = uvi[:-1]
            i = 0
            for x in uvi:
                uvi[i] = int(x)
                i += 1
        if ('Steps' in row[0]):
            steps = row[1].split(',')
            steps[0] = int(steps[0].strip('\n'))

f.close()


np.savetxt('co2.csv', co2, fmt='%d',delimiter = ',')
np.savetxt('voc.csv', voc, fmt='%d',delimiter = ',')
np.savetxt('uvi.csv', uvi, fmt='%d',delimiter = ',')
np.savetxt('steps.csv', steps, fmt='%d',delimiter = ',')
