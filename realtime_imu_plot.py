import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

# --- Settings ---
BAUD = 115200
MAX_POINTS = 200  # number of points to show live
PORT = None  # will auto-prompt

# --- Data Buffers ---
accel_x = deque(maxlen=MAX_POINTS)
accel_y = deque(maxlen=MAX_POINTS)
accel_z = deque(maxlen=MAX_POINTS)
gyro_x = deque(maxlen=MAX_POINTS)
gyro_y = deque(maxlen=MAX_POINTS)
gyro_z = deque(maxlen=MAX_POINTS)
time_axis = deque(maxlen=MAX_POINTS)
sample_count = 0

# --- Select COM port ---
def pick_port():
    ports = [p.device for p in serial.tools.list_ports.comports()]
    if not ports:
        print("No COM ports found.")
        return None
    print("Available COM ports:")
    for i, port in enumerate(ports):
        print(f"{i}: {port}")
    choice = input("Select COM port by number: ")
    try:
        return ports[int(choice)]
    except:
        print("Invalid selection.")
        return None

# --- Update Function for Plot ---
def update_plot(frame, ser, lines):
    global sample_count
    try:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line:
            parts = line.split(',')
            if len(parts) == 6:
                ax, ay, az, gx, gy, gz = map(float, parts)
                sample_count += 1
                accel_x.append(ax)
                accel_y.append(ay)
                accel_z.append(az)
                gyro_x.append(gx)
                gyro_y.append(gy)
                gyro_z.append(gz)
                time_axis.append(sample_count * 0.2)  # assuming 200ms interval

                # Update line data
                lines[0].set_data(time_axis, accel_x)
                lines[1].set_data(time_axis, accel_y)
                lines[2].set_data(time_axis, accel_z)
                lines[3].set_data(time_axis, gyro_x)
                lines[4].set_data(time_axis, gyro_y)
                lines[5].set_data(time_axis, gyro_z)

                for ax in [ax1, ax2]:
                    ax.relim()
                    ax.autoscale_view()

    except Exception as e:
        print(f"[ERROR] {e}")
    return lines

# --- Main ---
PORT = pick_port()
if not PORT:
    exit()

ser = serial.Serial(PORT, BAUD, timeout=1)
print(f"Connected to {PORT} at {BAUD} baud.")

# --- Setup Plot ---
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
ax1.set_title("Accelerometer (g)")
ax2.set_title("Gyroscope (°/s)")
ax2.set_xlabel("Time (s)")

line_ax = ax1.plot([], [], label="X")[0]
line_ay = ax1.plot([], [], label="Y")[0]
line_az = ax1.plot([], [], label="Z")[0]
line_gx = ax2.plot([], [], label="X")[0]
line_gy = ax2.plot([], [], label="Y")[0]
line_gz = ax2.plot([], [], label="Z")[0]

ax1.legend()
ax2.legend()
ax1.grid(True)
ax2.grid(True)

ani = animation.FuncAnimation(
    fig, update_plot, fargs=(ser, [line_ax, line_ay, line_az, line_gx, line_gy, line_gz]),
    interval=100, blit=False
)

plt.tight_layout()
plt.show()

ser.close()
print("Serial port closed.")
