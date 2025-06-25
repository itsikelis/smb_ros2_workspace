import csv
import matplotlib.pyplot as plt

# Load the centroids CSV
with open('centroids_output.csv', newline='') as csvfile:
    reader = csv.DictReader(csvfile)
    data = [row for row in reader]

# Convert data to dictionary grouped by class
centroids_dict = {}
for row in data:
    cls = row['class']
    x = float(row['x'])
    y = float(row['y'])
    if cls not in centroids_dict:
        centroids_dict[cls] = []
    centroids_dict[cls].append({'x': x, 'y': y})

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
