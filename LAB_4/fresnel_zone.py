import numpy as np
import plotly.graph_objects as go

#Enables plots in VSCode
import plotly.io as pio
pio.renderers.default = "vscode"

# Speed of Light 
Celerity = 300 * 10**(6) # m/s

# Fequency in Hz 
f_c2 = 2.4 * 10**(9) #
f_telemetry = 433 * 10**(6) 
f_video = 5.8 * 10**(9) 

# Wavelength in m 
lambda_c2 = Celerity / f_c2
lambda_telemtry = Celerity / f_telemetry
lambda_video = Celerity / f_video

# Choose frequency and wavelength to use

lam = lambda_video
f = f_video

# use matplotlib to create a 3D plot of fresnel zone fir each device 

def fresnel_axes(D: float, lam: float, n: int = 1):
    """
    Return (a, b, c) pour l'ellipsoïde de la n-ième Fresnel zone
    - a : grand demi-axe (le long de l'axe du lien)
    - b : petit demi-axe (rayon au milieu)
    - c : demi-distance focale (D/2)
    """
    c = D / 2.0
    a = (D + n * lam / 2.0) / 2.0
    b2 = a*a - c*c
    if b2 <= 0:
        # cas limite numérique : n trop grand vs D, renvoie b ~ 0
        b = 0.0
    else:
        b = np.sqrt(b2)
    return a, b, c


def rotation_matrix_from_z(u):
    """
    Matrice de rotation qui aligne l'axe z (0,0,1) vers le vecteur unitaire u.
    """
    z = np.array([0.0, 0.0, 1.0])
    u = u / np.linalg.norm(u)
    v = np.cross(z, u)
    s = np.linalg.norm(v)
    c = np.dot(z, u)
    if s == 0:  # déjà aligné
        return np.eye(3)
    vx = np.array([[0, -v[2], v[1]],
                   [v[2], 0, -v[0]],
                   [-v[1], v[0], 0]])
    R = np.eye(3) + vx + vx @ vx * ((1 - c) / (s**2))
    return R


def ellipsoid_mesh(a: float, b: float, center=(0,0,0), u_axis=(0,0,1), n_u=60, n_v=60, scale=1.0):
    """
    Maillage d'un ellipsoïde de révolution (b, b, a) orienté selon u_axis.
    - scale permet d'afficher 60% de la 1ère zone (scale=0.6).
    """
    u = np.linspace(0, 2*np.pi, n_u)
    v = np.linspace(0, np.pi, n_v)
    uu, vv = np.meshgrid(u, v)

    # ellipsoïde "brut" aligné sur z : (x,y,z) = (b sin v cos u, b sin v sin u, a cos v)
    xb = (b*scale) * np.sin(vv) * np.cos(uu)
    yb = (b*scale) * np.sin(vv) * np.sin(uu)
    zb = (a*scale) * np.cos(vv)

    # rotation pour aligner z -> u_axis
    R = rotation_matrix_from_z(np.array(u_axis))
    XYZ = np.stack([xb, yb, zb], axis=-1)  # (...,3)
    XYZ_rot = XYZ @ R.T

    # translation vers le centre
    cx, cy, cz = center
    X = XYZ_rot[..., 0] + cx
    Y = XYZ_rot[..., 1] + cy
    Z = XYZ_rot[..., 2] + cz
    return X, Y, Z

def make_mesh3d(X, Y, Z, color="royalblue", opacity=0.25, name="Fresnel"):
    # Conversion surface -> triangles (Plotly Mesh3d a besoin d'index; on triangule la grille)
    # On fabrique un maillage en rubans.
    nx, ny = X.shape
    verts = np.stack([X.ravel(), Y.ravel(), Z.ravel()], axis=1)
    def idx(i, j): return i*ny + j

    I = []
    J = []
    K = []
    for i in range(nx-1):
        for j in range(ny-1):
            # deux triangles par cellule (i,j)
            I += [idx(i, j), idx(i, j+1)]
            J += [idx(i+1, j), idx(i+1, j+1)]
            K += [idx(i+1, j+1), idx(i, j)]
    return go.Mesh3d(
        x=verts[:,0], y=verts[:,1], z=verts[:,2],
        i=I, j=J, k=K,
        color=color, opacity=opacity, name=name, lighting=dict(ambient=0.5)
    )

# -----------------------------
# Paramètres de la scène
# -----------------------------
# Positions (m) de l'émetteur et du récepteur (modifie-les à volonté)
TX = np.array([0.0, 0.0, 10.0])          # antenne au sol à 10 m
RX = np.array([1500.0, 300.0, 120.0])    # drone à ~1.5 km, en décalage latéral et altitude

# Exemple de fréquence : 433 MHz (télémetrie), change pour 2.4e9 ou 5.8e9

# Géométrie du lien
D_vec = RX - TX
D = np.linalg.norm(D_vec)
u_axis = D_vec / D
center = (TX + RX) / 2.0  # centre des ellipsoïdes

# Axes de Fresnel (zones 1 et 2)
a1, b1, c = fresnel_axes(D, lam, n=1)
a2, b2, _ = fresnel_axes(D, lam, n=2)

# Maillages (zone 1, zone 2 et "60% de la zone 1")
X1, Y1, Z1 = ellipsoid_mesh(a1, b1, center=center, u_axis=u_axis, n_u=80, n_v=60, scale=1.0)
X2, Y2, Z2 = ellipsoid_mesh(a2, b2, center=center, u_axis=u_axis, n_u=80, n_v=60, scale=1.0)
X6, Y6, Z6 = ellipsoid_mesh(a1, b1, center=center, u_axis=u_axis, n_u=80, n_v=60, scale=0.6)

# -----------------------------
# Scène Plotly
# -----------------------------
fig = go.Figure()

# Zone 2 (légèrement plus transparente)
fig.add_trace(make_mesh3d(X2, Y2, Z2, color="#A2C8FF", opacity=0.15, name="Zone de Fresnel n=2"))

# Zone 1
fig.add_trace(make_mesh3d(X1, Y1, Z1, color="#1f77b4", opacity=0.25, name="Zone de Fresnel n=1"))

# 60% de la 1ère zone (règle de clearance)
fig.add_trace(make_mesh3d(X6, Y6, Z6, color="#2ca02c", opacity=0.20, name="60% de la zone 1"))

# L’axe du lien
fig.add_trace(go.Scatter3d(
    x=[TX[0], RX[0]], y=[TX[1], RX[1]], z=[TX[2], RX[2]],
    mode="lines", line=dict(color="black", width=6), name="Ligne de visée"
))

# Marqueurs TX/RX
fig.add_trace(go.Scatter3d(x=[TX[0]], y=[TX[1]], z=[TX[2]],
                           mode="markers+text",
                           marker=dict(size=6, color="red"),
                           text=["TX"], textposition="top center", name="TX"))
fig.add_trace(go.Scatter3d(x=[RX[0]], y=[RX[1]], z=[RX[2]],
                           mode="markers+text",
                           marker=dict(size=6, color="orange"),
                           text=["RX"], textposition="top center", name="RX"))

# Mise en page
fig.update_layout(
    title=f"Zones de Fresnel 3D — f = {f/1e6:.1f} MHz, D = {D/1000:.2f} km, λ = {lam:.3f} m",
    scene=dict(
        xaxis_title="X (m)", yaxis_title="Y (m)", zaxis_title="Z (m)",
        aspectmode="data"
    ),
    legend=dict(bgcolor="rgba(255,255,255,0.7)")
)

fig.show()

fig.write_html("/home/delinm/Documents/Drone/LAB_4/fresnel_3d_video.html", auto_open=True)
print("Figure exportée dans fresnel_3d.html")


# Affichage du rayon au milieu (utile pour vérifier les ordres de grandeur)
r_milieu = np.sqrt(lam * D / 4.0)
print(f"Rayon de la 1ère zone au milieu du trajet : r1 = {r_milieu:.2f} m")

