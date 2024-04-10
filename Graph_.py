import pandas as pd
import matplotlib.pyplot as plt

# Replace 'Gen_Data/log3.csv' with the path to your CSV file
csv_file = 'Gen_Data/log1.csv'


# Load data
data = pd.read_csv(csv_file)
print(data)

# Filter data for the first 10 seconds
filtered_data = data[data.time < 10]

# Plot
plt.plot(filtered_data.time, filtered_data.rpm)
plt.title('RPM over Time (First 10 Seconds)')
plt.xlabel('Time (seconds)')
plt.ylabel('RPM')
plt.grid(True)
plt.show()
