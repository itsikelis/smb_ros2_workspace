import pandas as pd
import matplotlib.pyplot as plt

# Load the centroids CSV
df = pd.read_csv('centroids_output.csv')

# Convert to dictionary grouped by class
centroids_dict = {
    cls: group[['x', 'y']].to_dict(orient='records')
    for cls, group in df.groupby('class')
}

# Plot centroids, color-coded by class
plt.figure()

for cls, points in centroids_dict.items():
    xs = [point['x'] for point in points]
    ys = [point['y'] for point in points]
    plt.scatter(xs, ys, label=cls)

plt.title('Centroids by Class')
plt.xlabel('x')
plt.ylabel('y')
plt.legend()
plt.grid(True)
plt.show()
