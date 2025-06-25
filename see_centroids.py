import pandas as pd
import matplotlib.pyplot as plt

# Load the centroids CSV
df = pd.read_csv('centroids_output.csv')

# Get unique classes
classes = df['class'].unique()

# Plot centroids, color-coded by class
plt.figure()
for cls in classes:
    subset = df[df['class'] == cls]
    plt.scatter(subset['x'], subset['y'], label=cls)

plt.title('Centroids by Class')
plt.xlabel('x')
plt.ylabel('y')
plt.legend()
plt.grid(True)
plt.show()

