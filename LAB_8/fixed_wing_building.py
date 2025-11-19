import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator

m = 2.0 # kg
g = 9.81 # m/s²
W = m*g # N 
c = 0.18 # m
V =15.0 #m/s
rho = 1.2 #kg/m**3
mu = 1.69*10**(-5) # kg/(ms)

def main():
    # lift part 
    S = np.linspace(0.2,0.3,100) # area
    C_L = np.linspace(0.4,0.6,100) # cruise lift coefficient
    S, C_L = np.meshgrid(S,C_L)

    L_air = 0.5*rho*V**2* S* C_L 
    plt.figure(figsize=(8, 6))

# Filled contour map
    cs = plt.contourf(S, C_L, L_air, levels=40)

# Special contour at exactly 19.6 N
    plt.contour(S, C_L, L_air, levels=[W], colors='red', linewidths=3)

    plt.title("2D Contour Map of Lift\nRed Line = Lift = 19.6 N (Level Cruise)")
    plt.xlabel("Wing Area S (m²)")
    plt.ylabel("Lift Coefficient C_L")

    cbar = plt.colorbar(cs)
    cbar.set_label("Lift (N)")

    plt.grid(True, linestyle="--", alpha=0.3)
    plt.show()

    # reynold part 
if __name__ == "__main__":
    main()

