#!/usr/bin/env python3
import trimesh
import numpy as np
from pathlib import Path

# Percorso al file STL relativo al pacchetto
stl_path = Path(__file__).resolve().parent.parent / "description/mesh/body.STL"

# Carica la mesh
mesh = trimesh.load_mesh(stl_path)

# Densità del materiale (PLA)
density = 1000  # kg/m^3

# Calcola le proprietà di massa
mesh.density = density
mass_props = mesh.mass_properties
I = mass_props['inertia']
com = mass_props['center_mass']

# Calcola autovalori e autovettori (momenti e assi principali)
eigvals, eigvecs = np.linalg.eigh(I)

print("Momenti principali d'inerzia [kg·m²]:", eigvals)
print("Assi principali (colonne):\n", eigvecs)

# Crea una scena
scene = trimesh.Scene()

# Aggiungi la mesh
scene.add_geometry(mesh)

# Aggiungi il punto del centro di massa
com_marker = trimesh.creation.icosphere(radius=mesh.scale * 0.01)
com_marker.apply_translation(com)
scene.add_geometry(com_marker)

# Aggiungi vettori degli assi principali
scale = mesh.scale * 0.1  # per visibilità
colors = [(255, 0, 0, 255), (0, 255, 0, 255), (0, 0, 255, 255)]

for i in range(3):
    direction = eigvecs[:, i]
    line = trimesh.load_path(np.array([
        com,
        com + direction * scale
    ]))
    line.colors = np.tile(colors[i], (len(line.entities), 1))
    scene.add_geometry(line)

# Mostra tutto
scene.show()
