import os
import csv
import math
from typing import List


def load_csv_in_radians(path: str) -> List[List[float]]:
    """
    Carica un CSV dove ogni riga contiene gli angoli (in gradi) delle giunzioni
    e li converte in radianti.

    Ritorna: lista di liste di float.
    """
    if not os.path.exists(path):
        raise FileNotFoundError(f"CSV non trovato: {path}")

    positions: List[List[float]] = []
    with open(path, "r") as f:
        reader = csv.reader(f)
        for row in reader:
            if not row:
                continue
            degrees = [float(v) for v in row]
            radians = [math.radians(v) for v in degrees]
            positions.append(radians)

    return positions