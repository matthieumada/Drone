import numpy as np
import matplotlib.pyplot as plt

# Params
h0 = 0.5
L = 400.0
height = 50.0
c = 3.0e8

def fresnel(f, d1, L_total, zone_number=1):
    d2 = L_total - d1
    lam = c / f
    return np.sqrt((zone_number * lam * d1 * d2) / (d1 + d2))

# --- input
f_mhz = float(input("Frequency (MHz): "))
f = f_mhz * 1e6

# Distances
N = 1000
D = np.linspace(0, L, N)

# LOS and Fresnel boundaries
LOS = h0 + (height - h0) * (D / L)
R_vec = np.array([fresnel(f, d, L) for d in D])
C_up = LOS + R_vec
C_down = LOS - R_vec

# Choose point (milieu = 200 m)
d_choice = 200.0
idx = np.argmin(np.abs(D - d_choice))
d_pt = D[idx]
R_pt = R_vec[idx]
LOS_pt = LOS[idx]

# Set building height = LOS at ce point
h_obs_at_point = LOS_pt

# clearance et blockage (clip 0..1)
clearance = LOS_pt - h_obs_at_point            # = 0 si égalité
blockage = (R_pt - clearance) / R_pt
blockage = np.clip(blockage, 0.0, 1.0)         # s'assure que c'est entre 0 et 1

# Print résultats
print(f"Point d = {d_pt} m")
print(f"Rayon 1ère zone Fresnel R = {R_pt:.3f} m")
print(f"LOS à ce point = {LOS_pt:.3f} m")
print(f"Hauteur obstacle fixée = {h_obs_at_point:.3f} m")
print(f"Clearance = {clearance:.3f} m")
print(f"Blocage = {blockage*100:.1f}%")

# Tracé
plt.figure(figsize=(10,6))
plt.plot(D, LOS, '--', label='LOS')
plt.plot(D, C_up, '-', label='Limite sup. 1ère Fresnel')
plt.plot(D, C_down, ':', label='Limite inf. 1ère Fresnel')

# obstacle: on met une petite largeur centrée sur d_choice au niveau LOS
obs_width = 20.0
obs = np.zeros_like(D)
left = d_choice
right = d_choice + obs_width
obs[(D >= left) & (D <= right)] = h_obs_at_point
plt.plot(D, obs, color='saddlebrown', linewidth=3, label='Bâtiment (hauteur = LOS)')

plt.scatter([0, L], [h0, height], color=['orange','purple'])
plt.axvline(d_pt, color='gray', linestyle='--')
plt.ylim(0, max(height, LOS_pt + R_pt) + 5)
plt.xlabel('Distance (m)'); plt.ylabel('Hauteur (m)')
plt.title(f'Blocage à d={d_pt} m — f={f_mhz} MHz')
plt.legend(); 
plt.show()
