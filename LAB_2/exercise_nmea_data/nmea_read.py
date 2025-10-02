#!/usr/bin/env python3
#/****************************************************************************
# nmea read function
# Copyright (c) 2018-2020, Kjeld Jensen <kj@kjen.dk>
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
#****************************************************************************/
'''
2018-03-13 Kjeld First version, that reads in CSV datacd 
2020-02-03 Kjeld Python 3 compatible
2020-09-17 Kjeld Changed first line to python3
'''
import matplotlib.pyplot as plt

class nmea_class:
  def __init__(self):
    self.data = []

  def import_file(self, file_name):
    file_ok = True
    try:
      # read all lines from the file and strip \n
      lines = [line.rstrip() for line in open(file_name)] 
    except:
      file_ok = False
    if file_ok == True:
      pt_num = 0
      for i in range(len(lines)): # for all lines
        if len(lines[i]) > 0 and lines[i][0] != '#': # if not a comment or empty line
          csv = lines[i].split (',') # split into comma separated list
          self.data.append(csv)

  def print_data(self):
    for i in range(len(self.data)):
      if (self.data[i][0] == '$GPGGA') :
        print (self.data[i][7])
  
  def extract(self):
    Time = []
    Lat = []
    Long = []
    satellite = []
    Alti = []
    for i in range(len(self.data)):
      if (self.data[i][0] == '$GPGGA') :
        Time.append(float(self.data[i][1])* 10**(-4))
        Lat.append(float(self.data[i][2]))
        Long.append(float(self.data[i][4]))
        satellite.append(float(self.data[i][7]))
        Alti.append(float(self.data[i][9]))
        print(" altitude" ,float(self.data[i][9]) )
    return Time, Lat, Long, satellite, Alti

  def choosen(self, index):
    Time = []
    choose = []
    for i in range(len(self.data)):
      if (self.data[i][0] == '$GPGGA') :
        Time.append(float(self.data[i][1])* 10**(-4))
        choose.append(float(self.data[i][index]))
        print(float(self.data[i][index]))
    return Time, choose


if __name__ == "__main__":
  print ('Importing file')
  nmea = nmea_class()
  nmea.import_file ('nmea_trimble_gnss_eduquad_flight.txt')
  #nmea.print_data()
  
  Time, Lat, Long, satellite, Alti = nmea.extract()

  # Altitude above sea level over the time 
  plt.figure
  plt.plot(Time,Alti, label = 'Altitude')
  plt.xlabel('Time [h]')
  plt.ylabel('Altitude [m]')
  plt.title('Altitude over time')
  plt.legend()
  plt.grid('on')

  # Number of satellites over the time
  plt.figure()
  plt.plot(Time,satellite, label = 'Satellites')
  plt.xlabel('Time [h]')
  plt.ylabel('Number of satellites')
  plt.title('Number of satellites over time')
  plt.legend()
  plt.grid('on') 

  # Maps of the flight 
  fig = plt.figure()
  ax = fig.add_subplot(111, projection='3d')

  # Tracé
  ax.plot(Lat, Long, Alti, label='Postion')
  ax.set_xlabel('Latitude [deg]')
  ax.set_ylabel('Longitude [deg]')
  ax.set_zlabel('Altitude [m]')
  ax.legend()

### Flight during 24h
  # Loading data from another file 
  nmea2 = nmea_class()
  nmea2.import_file ('nmea_ublox_neo_24h_static.txt') 
  # GNSS accuracy plotting 
  time, acc = nmea2.choosen(8) # index 6 is the horizontal accuracy
  #nmea2.print_data()
  plt.figure()
  plt.plot(time,acc, label = 'Horizontal accuracy')
  plt.xlabel('Time [h]')
  plt.ylabel('Horizontal accuracy')
  plt.title('Horizontal accuracy over time')
  plt.legend()
  plt.grid('on')
  plt.show()
 
