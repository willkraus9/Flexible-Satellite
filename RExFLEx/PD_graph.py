import csv
import matplotlib.pyplot as plt

csv_file_path = 'RExFLEx\RExFLEx_Data - Sheet1.csv'

time = range(0,2026,1)
Xpos_with = []
vel_with = []
Xpos_without = []

with open(csv_file_path, 'r') as csvfile:
    csvreader = csv.reader(csvfile)
    # Skip the header row
    next(csvreader)
    # Read each row of data
    for row in csvreader:
        Xpos_with.append((row[0]))
        # vel_with.append((row[1]))
        # Xpos_without.append((row[2]))
        # time.append((row[4]))

plt.figure()

plt.xlabel('Timesteps')
# plt.ylabel('Displacement, in cm')
plt.title('Hardware PD Control Proof-of-Concept')

plt.plot(time, Xpos_with, color = "blue",  label='With Reaction Wheel: Distance from Origin (cm)')
# plt.plot(time, Xpos_without, color = "black", linestyle = "dashed", label='Without Reaction Wheel: Distance from Origin (cm)')
# plt.plot(time, vel_with, label='Angular Velocity Estimate (rotations/second)', color='green')

plt.legend()
plt.show()
