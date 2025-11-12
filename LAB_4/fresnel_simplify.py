# This file is to build 2D plot of fresnel zone more easier to put on drive. 

import numpy as np
import matplotlib.pyplot as plt

L = 5.0 #m 
c = 3.0 * 10 **(8) # m/s
height = 4.0 #m

def fresnel(f,d1, zone_number):
    d2 = L - d1
    lamb = c / f # m
    R = np.sqrt((zone_number * lamb * d1 * d2) / (d1 + d2))
    return R 

if __name__ == "__main__":
    C2 = []
    C2_bis = []
    f_C2 = 2400 * 10**(6) # Hz

    Telemetry = []
    Telemetry_bis = []
    f_telemetry = 433  * 10**(6) # Hz

    Video = []
    Video_bis = []
    f_video = 5800 * 10**(6) # Hz

    Line_of_sight = []
    D = np.linspace(0,L,300)

    for i in D:
        # C2 
        C2.append(fresnel(f_C2,i,1)+ height )
        C2_bis.append(height - fresnel(f_C2,i,1))

        # Telemetry
        Telemetry.append(fresnel(f_telemetry,i,1) + height)
        Telemetry_bis.append(height - fresnel(f_telemetry,i,1))
                             
        #video 
        Video.append(fresnel(f_video,i,1) + height)
        Video_bis.append(height - fresnel(f_video,i,1) )

        Line_of_sight.append(height)
        
plt.figure()
plt.title("fresnel zone number 1 fir a distance of " + str(L)+ "m and a height:"+ str(height)+'m')
plt.xlabel("Distance [m]")
plt.ylabel(" Height [m]")
plt.scatter(0,height, label = 'Emitter', color = 'orange', s = 150)
plt.scatter(L, height, label = 'Receipter', color = 'purple', s = 150)
plt.plot(D,Line_of_sight, color = 'black', label = 'Line of Sight')
plt.plot(D,C2, color = 'red',label = 'C2: '+ str(2400)+ 'MHz')
plt.plot(D,C2_bis, color = 'red')
plt.plot(D,Telemetry, color = 'blue', label = 'Telemetry: ' + str(433)+ 'MHz')
plt.plot(D,Telemetry_bis, color = 'blue')
plt.plot(D, Video, color = 'green' , label = 'Video: ' + str(5800)+ 'MHz')
plt.plot(D, Video_bis, color = 'green')
plt.legend()
plt.show()
