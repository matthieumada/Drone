#!/usr/bin/env python3
#*****************************************************************************
# UTM projection conversion test
# Copyright (c) 2013-2020, Kjeld Jensen <kjeld@frobomind.org>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#*****************************************************************************
"""
This file contains a simple Python script to test the UTM conversion class.

Revision
2013-04-05 KJ First version
2015-03-09 KJ Minor update of the license text.
2016-01-16 KJ Corrected a minor problem with the library location reference.
2020-02-03 KJ Python 3 compatible, changed functionality: it now moves a
              geodetic position 100 meter East
2020-09-17 KJ Changed first line to python3
"""

# import utmconv class
from utm import utmconv
import math as m
R = 6371.0 # earth radius in km

def Gcircle_dist(lat1,long1,lat2,long2):
    # Great circle distance between two geodetic coordinates
    lat1 = lat1 * m.pi / 180.0
    long1 = long1 * m.pi / 180.0
    lat2 = lat2 * m.pi / 180.0
    long2 = long2 * m.pi / 180.0
    d_radian = 2 * m.asin(m.sqrt((m.sin((lat1-lat2)/2))**2 + m.cos(lat1)*m.cos(lat2)*(m.sin((long1-long2)/2))**2))
    return d_radian * R

# geodetic reference coordinate
lat1 =  55.47
lon1 = 010.33

print ('First position [deg]:')
print ('  latitude:  %.8f'  % (lat1))
print ('  longitude: %.8f'  % (lon1))

# instantiate utmconv class
uc = utmconv()

# convert from geodetic to UTM
(hemisphere, zone, letter, e1, n1) = uc.geodetic_to_utm (lat1,lon1)
print ('\nConverted from geodetic to UTM [m]')
print ('  %d %c %.5fe %.5fn' % (zone, letter, e1, n1))

# now generating the second UTM coordinate
e2 = e1 + 100.0 
n2 = n1 #+ 100.0

# convert back from UTM to geodetic
(lat2, lon2) = uc.utm_to_geodetic (hemisphere, zone, e2, n2)
print ('\nSecond position 100 meter East [deg]:')
print ('  latitude:  %.8f'  % (lat2))
print ('  longitude: %.8f'  % (lon2))

# compute the great circle distance between the two geodetic coordinates
d = Gcircle_dist(lat1,lon1,lat2,lon2)
print("Distance between the two geodetic coordinates [km]: %.8f" % (d))

HAC_lat = 55.47182
HAC_long = 10.32581

SVanne_lat = 55.12518
SVanne_long = 10.25419
# distance between HAC and Svanne [km]:
d_hac_svanne = Gcircle_dist(HAC_lat,HAC_long,SVanne_lat,SVanne_long)
print("Distance between HAC and Svanne [km]: %.8f" % (d_hac_svanne))