#!/usr/bin/env python2

import utm
import time


t0 = time.time()
coord1 = (50.365370, -4.145806)
coord2 = (50.365370, -4.16)#50.374927, -4.152902

utm1 = utm.from_latlon(coord1[0],coord1[1])
utm2 = utm.from_latlon(coord2[0],coord2[1])
dx, dy = utm1[0]-utm2[0],utm1[1]-utm2[1]
length = (dx**2+dy**2)**0.5
print(length,dx,dy,time.time()-t0)