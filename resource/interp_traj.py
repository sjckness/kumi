import numpy as np
import pandas as pd

def interpolate_csv(input_file, output_file, y):
    # Legge il CSV
    df = pd.read_csv(input_file)

    # Numero di righe originali
    n = len(df)

    # Vettore "asse x" originale (indice normalizzato)
    x_original = np.linspace(0, 1, n)

    # Nuovo asse con y punti
    x_new = np.linspace(0, 1, y)

    # DataFrame per salvare i dati interpolati
    df_interpolated = pd.DataFrame()

    # Interpola ogni colonna
    for col in df.columns:
        df_interpolated[col] = np.interp(x_new, x_original, df[col])

    # Salva il nuovo CSV
    df_interpolated.to_csv(output_file, index=False)

    print(f"Creato file interpolato con {y} righe e {len(df.columns)} colonne.")


# ESEMPIO USO
if __name__ == "__main__":
    input_csv = "/home/andreas/dev_ws/src/kumi/resource/demo_flip.csv"
    output_csv = "/home/andreas/dev_ws/src/kumi/resource/output_interpolato.csv"
    y = 500  # numero di righe desiderato

    interpolate_csv(input_csv, output_csv, y)