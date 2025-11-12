import numpy as np
import matplotlib.pyplot as plt

# Fixed Parameters 
h0 = 0.5     # Transmitter height  (m)
L = 400.0    # Total distance  (m)
height = 50.0 # Drone height  (m)
c = 3.0 *10**(8)   # speed of light  (m/s)

def fresnel(f, d1, L_total, zone_number=1):
    d2 = L_total - d1
    lamb = c / f
    R = np.sqrt((zone_number * lamb * d1 * d2) / (d1 + d2))
    return R

def make_obstacle_profile(N, D, pos_center, width, h_top):
    """
    Obstacle vector 
    - N : Point number 
    - D : DIstance vector 
    - pos_center : center point of the obstacle 
    - width : total width
    - h_top : height of the top of the obstacle 
    """
    obs = np.zeros(N)
    left = pos_center - width/2.0
    right = pos_center + width/2.0
    for i, x in enumerate(D):
        if left <= x <= right:
            obs[i] = h_top
        else:
            obs[i] = 0.0
    return obs

if __name__ == "__main__":
    name = input("Technology desired : ")
    f_mhz = float(input("Frequency of the technology (MHz) : "))
    f_hz = f_mhz * 1e6

    # Obstacle parameter 
    desired_block = 0.40    #  in percentage 
    obstacle_position = L/2 # obstacle position  (m) - middle 
    obstacle_width = 20.0   # width of obstacle (m) - modifiable

    # distance vector 
    N = 1000
    D = np.linspace(0, L, N)

    #Computation of Fresnel zoen limits 
    C_up = []
    C_down = []
    LOS = []
    for d in D:
        R = fresnel(f_hz, d, L, 1)
        los_h = h0 + (height - h0) * (d / L)
        C_up.append(los_h + R)
        C_down.append(los_h - R)
        LOS.append(los_h)
    C_up = np.array(C_up)
    C_down = np.array(C_down)
    LOS = np.array(LOS)

    # Computation at the desired point 
    d_idx = np.argmin(np.abs(D - obstacle_position))
    dpt = D[d_idx]
    R_pt = fresnel(f_hz, dpt, L, 1)
    los_pt = LOS[d_idx]

    # height of the blockage 
    h_obs_required = los_pt - (1 - desired_block) * R_pt

    # si h_obs_required < 0 -> impossible (sous le sol) -> on clamp à 0
    if h_obs_required < 0:
        h_obs_required = 0.0

    # création du profil d'obstacle (plateau)
    obstacle_profile = make_obstacle_profile(N, D, obstacle_position, obstacle_width, h_obs_required)

    # calcul réel du blockage avec cet obstacle
    clearance = los_pt - h_obs_required
    blockage_fraction = max(0.0, (R_pt - clearance) / R_pt)

    # résultats numériques
    print("\n--- result " + name +"à",f_mhz, " MHz ---")
    print("Position of obstacle desired  :", dpt, " m (sur ",L," m)")
    print("Radius of the Fresnel zone at this point :", R_pt," m")
    print("Height of Line of sight :",los_pt, "m")
    print("Height of obstace for the blockage", desired_block*100, ":", h_obs_required," m")
    print("Blockage computated with this obstacle  : {blockage_fraction*100:.2f}%")
    print("(Obstacle width  =",obstacle_width, "m)\n")

    # tracé
    plt.figure(figsize=(11,6))
    plt.title(f"First zone of Fresnel for {name} at {f_mhz} MHz")
    plt.xlabel("Distance [m]")
    plt.ylabel("Height [m]")

    plt.plot(D, LOS, '--', color='black', label='Line of sight')
    plt.plot(D, C_up, '-', color='red', label='Upper limit of 1rd Fresnel zone')
    plt.plot(D, C_down, ':', color='red', label='Lower limit of 1rd Frensnel zone')
    plt.plot(D, obstacle_profile, color='saddlebrown', linewidth=3, label='Obstacle')

    plt.scatter([0, L], [h0, height], color=['orange','purple'], s=120)
    plt.text(0, h0+0.5, 'Transmitter', fontsize=9)
    plt.text(L, height+0.5, 'Drone (RX)', fontsize=9)

    # ligne verticale pour la position d'obstacle + annotation
    plt.axvline(x=dpt, color='gray', linestyle='--', linewidth=0.8)
    plt.annotate(f"Obstacle Point\nR = {R_pt:.2f} m\nLOS = {los_pt:.2f} m\nh_obs = {h_obs_required:.2f} m\nBlockage = {blockage_fraction*100:.1f}%",
                 xy=(dpt, h_obs_required),
                 xytext=(dpt + L*0.02, max(h_obs_required, los_pt) + 3),
                 arrowprops=dict(arrowstyle='->', color='gray'),
                 fontsize=9, bbox=dict(boxstyle="round,pad=0.3", fc="w"))

    # sol
    plt.axhline(y=0, color='green', linestyle='-', label='Sol')
    plt.ylim(bottom=0)
    plt.xlim(0, L)
    plt.grid(True)
    plt.legend()
    plt.show()
