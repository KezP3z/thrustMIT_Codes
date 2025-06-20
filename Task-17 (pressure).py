import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

df = pd.read_csv('FML.csv')


velocity_array = df['velocity'].to_numpy()
velocitypoints=np.array(velocity_array)

ax_array=df['acc_x'].to_numpy()
ax_points=np.array(ax_array)

ay_array=df['acc_y'].to_numpy()
ay_points=np.array(ay_array)

az_array=df['acc_z'].to_numpy()
az_points=np.array(az_array)

time_array = df['time'].to_numpy()
timepoints=np.array(time_array)

acct_array = df['acc_tot'].to_numpy()
acct_points=np.array(acct_array)

press_array=df['pressure'].to_numpy()
press_points= np.array(press_array)

window_size = 17  # Choose window size
df['pressure_smoothed'] = df['pressure'].rolling(window=window_size, center=False).mean()

press_array=df['pressure_smoothed'].to_numpy()
press_points= np.array(press_array)

#plt.plot(timepoints,velocitypoints)
#plt.plot(timepoints, acct_points)
#plt.plot(timepoints,press_points)

min_pressure = df['pressure_smoothed'].min()
print(min_pressure)


min_pressure_index = df['pressure_smoothed'].idxmin()
apogee = df.loc[min_pressure_index, 'altitude']
tome = df.loc[min_pressure_index, 'time']
print ("Time = ", tome )

print("apogee=", apogee)


print("apogee reached")

plt.plot(timepoints, df['pressure_smoothed'])
plt.show()
