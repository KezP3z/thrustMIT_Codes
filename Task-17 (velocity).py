import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

df = pd.read_csv('FML.csv')

#df = df.iloc[236:].reset_index(drop=True)

net_a = df['acc_tot'].to_numpy()
net_a_points = np.array(net_a)

window_size = 10
df['smooth_acc'] = df['acc_tot'].rolling(window=window_size, center=False).mean()

df['is_apogee'] = (df['velocity'] < 0) & (df['velocity'].shift(1) >= 0)


apogee_index = df[df['is_apogee']].index[0]
apogee_time = df.loc[apogee_index, 'time']
print(f"Apogee detected at time: {apogee_time:.2f} seconds")
apogee = df.loc[apogee_index, 'altitude']
print('apogee = ', apogee)

plt.plot(df['time'], df['velocity'], label='Velocity', color='blue')
plt.plot(df['time'], df['smooth_acc'], label='Smoothed Acceleration', color='orange')
plt.plot(df['time'], df['altitude'], label='alitude', color='green')

if df['is_apogee'].any():
    plt.axvline(x=apogee_time, color='red', linestyle='--', label=f'Apogee at {apogee_time:.2f}s')

plt.title('Apogee Detection Using Velocity and Acceleration')
plt.xlabel('Time (s)')
plt.ylabel('Value')
plt.legend()
plt.grid()
plt.show()
