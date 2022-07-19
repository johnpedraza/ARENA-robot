import matplotlib.pyplot as plt
import pandas as pd

ax = plt.axes(projection="3d")

optitrack_x_data = []
optitrack_y_data = []
optitrack_z_data = []

optitrack_df = pd.read_csv('recording_optitrack.csv')

for pos in optitrack_df['position']:
    optitrack_x_data.append(float(pos.strip('][').split(', ')[0]))
    optitrack_y_data.append(float(pos.strip('][').split(', ')[1]))
    optitrack_z_data.append(float(pos.strip('][').split(', ')[2]))

vio_x_data = []
vio_y_data = []
vio_z_data = []

vio_df = pd.read_csv('recording_vio.csv')

for pos in vio_df['position']:
    vio_x_data.append(float(pos.strip(')(').split(', ')[0]))
    vio_y_data.append(float(pos.strip(')(').split(', ')[1]))
    vio_z_data.append(float(pos.strip(')(').split(', ')[2]))

apriltags_x_data = []
apriltags_y_data = []
apriltags_z_data = []

apriltags_df = pd.read_csv('recording_apriltags.csv')

for pos in apriltags_df['position']:
    if pos == pos: # check for NaN
        apriltags_x_data.append(float(pos.strip('][').split(', ')[0]))
        apriltags_y_data.append(float(pos.strip('][').split(', ')[1]))
        apriltags_z_data.append(float(pos.strip('][').split(', ')[2]))

# ax.scatter(optitrack_x_data, optitrack_y_data, optitrack_z_data)
# ax.scatter(vio_x_data, vio_y_data, vio_z_data)
ax.scatter(apriltags_x_data, apriltags_y_data, apriltags_z_data)

plt.show()