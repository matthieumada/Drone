import numpy as np
import matplotlib.pyplot as plt

# Paramètres
h0 = 0.5  # Hauteur de l'émetteur (m)
L = 400.0  # Distance totale (m)
height = 50.0  # Hauteur du drone (m)
c = 3.0 * 10**8  # Vitesse de la lumière (m/s)

def fresnel(f, d1, zone_number):
    d2 = L - d1
    lamb = c / f
    R = np.sqrt((zone_number * lamb * d1 * d2) / (d1 + d2))
    return R

if __name__ == "__main__":
    name = input("Nom de la technologie : ")
    f = float(input("Fréquence de la technologie (MHz) : "))
    f_hz = f * 10**6  # Conversion en Hz

    # Vecteur de distances
    D = np.linspace(0, L, 1000)

    # Calcul des limites de la zone de Fresnel
    C2_upper = []
    C2_lower = []
    Line_of_sight = []

    for d in D:
        R = fresnel(f_hz, d, 1)
        # Équation de la ligne de visée : y = h0 + (height - h0) * (d / L)
        los_height = h0 + (height - h0) * (d / L)
        C2_upper.append(los_height + R)
        C2_lower.append(los_height - R)
        Line_of_sight.append(los_height)

    # Vérification du blocage à mi-chemin
    mid_index = len(D) // 2
    mid_distance = D[mid_index]
    R_mid = fresnel(f_hz, mid_distance, 1)
    los_height_mid = h0 + (height - h0) * (mid_distance / L)
    clearance = los_height_mid - 0  # Distance entre la ligne de visée et le sol
    blockage_percentage = (R_mid - clearance) / R_mid

    # Affichage des résultats
    print(f"Rayon de la première zone de Fresnel à mi-chemin : {R_mid:.2f} m")
    print(f"Hauteur de la ligne de visée à mi-chemin : {los_height_mid:.2f} m")
    print(f"Pourcentage de blocage par le sol : {blockage_percentage * 100:.1f}%")
    print(f"Blocage significatif (>40%) : {'Oui' if blockage_percentage > 0.4 else 'Non'}")

    # Tracé du graphique
    plt.figure(figsize=(10, 6))
    plt.title(f"Première zone de Fresnel pour {name} à {f} MHz")
    plt.xlabel("Distance [m]")
    plt.ylabel("Hauteur [m]")

    # Tracé des points et des courbes
    plt.scatter(0, h0, label='Émetteur', color='orange', s=150)
    plt.scatter(L, height, label='Récepteur (Drone)', color='purple', s=150)
    plt.plot(D, Line_of_sight, color='black', linestyle='--', label='Ligne de visée')
    plt.plot(D, C2_upper, color='red', label='Limite supérieure de la zone de Fresnel')
    plt.plot(D, C2_lower, color='red', linestyle=':', label='Limite inférieure de la zone de Fresnel')

    # Tracé du sol (y = 0)
    plt.axhline(y=0, color='green', linestyle='-', label='Sol')

    # Annotations
    if blockage_percentage > 0.4:
        plt.annotate(f"Blocage critique : {blockage_percentage * 100:.1f}%",
                     xy=(mid_distance, 0),
                     xytext=(mid_distance, 5),
                     arrowprops=dict(facecolor='red', shrink=0.05),
                     fontsize=10, color='red')

    plt.legend()
    plt.grid(True)
    plt.show()