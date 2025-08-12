import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV into a DataFrame
df = pd.read_csv("./FK/trajectory.csv")

# Plot Position vs Time
plt.figure(figsize=(8,4))
plt.plot(df['t'], df['x'], label='x')
plt.plot(df['t'], df['y'], label='y')
plt.plot(df['t'], df['z'], label='z')
plt.title('Foot Position vs Time')
plt.xlabel('Time (s)')
plt.ylabel('Position (m)')
plt.legend()
plt.grid(True)
plt.tight_layout()

# Plot Velocity vs Time
plt.figure(figsize=(8,4))
plt.plot(df['t'], df['vx'], label='vx')
plt.plot(df['t'], df['vy'], label='vy')
plt.plot(df['t'], df['vz'], label='vz')
plt.title('Foot Velocity vs Time')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.legend()
plt.grid(True)
plt.tight_layout()

# Plot Acceleration vs Time
plt.figure(figsize=(8,4))
plt.plot(df['t'], df['ax'], label='ax')
plt.plot(df['t'], df['ay'], label='ay')
plt.plot(df['t'], df['az'], label='az')
plt.title('Foot Acceleration vs Time')
plt.xlabel('Time (s)')
plt.ylabel('Acceleration (m/s²)')
plt.legend()
plt.grid(True)
plt.tight_layout()

plt.show()
