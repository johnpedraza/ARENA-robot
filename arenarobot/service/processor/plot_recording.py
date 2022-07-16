import matplotlib.pyplot as plt
import pandas as pd

ax = plt.axes(projection="3d")

x_data = []
y_data = []
z_data = []

df_recorded = pd.read_csv('recording.csv')

for pos in df_recorded['position']:
    x_data.append(float(pos.strip('][').split(', ')[0]))
    y_data.append(float(pos.strip('][').split(', ')[1]))
    z_data.append(float(pos.strip('][').split(', ')[2]))

ax.scatter(x_data, y_data, z_data)
plt.show()