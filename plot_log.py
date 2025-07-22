import pandas as pd
import matplotlib.pyplot as plt
from tkinter import filedialog, Tk

def pick_csv_file():
    root = Tk()
    root.withdraw()
    return filedialog.askopenfilename(title="Select IMU CSV Log", filetypes=[("CSV Files", "*.csv")])

def main():
    file_path = pick_csv_file()
    if not file_path:
        print("No file selected.")
        return

    try:
        df = pd.read_csv(file_path)
    except Exception as e:
        print(f"Error reading file: {e}")
        return

    expected_cols = ['Accel_X', 'Accel_Y', 'Accel_Z', 'Gyro_X', 'Gyro_Y', 'Gyro_Z']
    if not all(col in df.columns for col in expected_cols):
        print("CSV missing expected columns. Expected header: Accel_X,Accel_Y,Accel_Z,Gyro_X,Gyro_Y,Gyro_Z")
        return

    df['Time'] = df.index * 0.2  # assuming 200ms sampling interval

    fig, axs = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    axs[0].plot(df['Time'], df['Accel_X'], label='Accel_X')
    axs[0].plot(df['Time'], df['Accel_Y'], label='Accel_Y')
    axs[0].plot(df['Time'], df['Accel_Z'], label='Accel_Z')
    axs[0].set_ylabel('Accel (g)')
    axs[0].legend()
    axs[0].grid(True)

    axs[1].plot(df['Time'], df['Gyro_X'], label='Gyro_X')
    axs[1].plot(df['Time'], df['Gyro_Y'], label='Gyro_Y')
    axs[1].plot(df['Time'], df['Gyro_Z'], label='Gyro_Z')
    axs[1].set_xlabel('Time (s)')
    axs[1].set_ylabel('Gyro (°/s)')
    axs[1].legend()
    axs[1].grid(True)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
