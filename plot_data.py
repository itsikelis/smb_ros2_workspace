import pandas as pd
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans
import numpy as np

# Load data
df = pd.read_csv('points_transformed.csv')
classes = df['class'].unique()

# List to collect centroids
centroid_rows = []

# Process each class
for cls in classes:
    print(f"\nShowing scatter plot for class '{cls}'. Close the window to proceed to clustering...")

    subset = df[df['class'] == cls][['x', 'y']]

    # Plot raw data
    plt.figure()
    plt.scatter(subset['x'], subset['y'])
    plt.title(f'Raw Data for Class: {cls}')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.grid(True)
    plt.show()

    # Get user input for number of clusters
    while True:
        try:
            k = int(input(f"Enter number of clusters (k) for class '{cls}': "))
            if k <= 0:
                raise ValueError("k must be greater than 0.")
            break
        except ValueError as e:
            print(f"Invalid input: {e}. Please try again.")

    # Run K-Means
    kmeans = KMeans(n_clusters=k, random_state=0)
    labels = kmeans.fit_predict(subset)
    centers = kmeans.cluster_centers_

    # Store centroids
    for center in centers:
        centroid_rows.append({'class': cls, 'x': center[0], 'y': center[1], 'z': np.nan})

    # Plot clustered data
    plt.figure()
    plt.scatter(subset['x'], subset['y'], c=labels, cmap='tab10')
    plt.scatter(centers[:, 0], centers[:, 1], color='black', marker='x', s=100, label='Centroids')
    plt.title(f'K-Means Clustering for Class: {cls} (k={k})')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.legend()
    plt.grid(True)
    plt.show()

# Save all centroids to CSV
centroids_df = pd.DataFrame(centroid_rows)
centroids_df.to_csv('centroids_output.csv', index=False)
print("\nCentroids saved to 'centroids_output.csv'")

