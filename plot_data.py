import csv
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans
import numpy as np

# Load data
with open("points_transformed.csv", newline="") as csvfile:
    reader = csv.DictReader(csvfile)
    df = [row for row in reader]

# Convert values to correct types and collect unique classes
for row in df:
    row["x"] = float(row["x"])
    row["y"] = float(row["y"])
    row["confidence"] = float(row["confidence"])
    row["class"] = row["class"]

classes = list(set(row["class"] for row in df))

# List to collect centroids
centroid_rows = []

# Process each class
for cls in classes:
    print(
        f"\nShowing scatter plot for class '{cls}'. Close the window to proceed to clustering..."
    )
    subset = [row for row in df if row["class"] == cls]
    x_vals = [row["x"] for row in subset]
    y_vals = [row["y"] for row in subset]
    confidence_vals = [row["confidence"] for row in subset]

    # Plot raw data with confidence-based transparency
    plt.figure(figsize=(40, 40))
    plt.scatter(x_vals, y_vals, alpha=confidence_vals)
    plt.title(f"Raw Data for Class: {cls}")
    plt.xlabel("x")
    plt.ylabel("y")
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
    data_array = np.array([[row["x"], row["y"]] for row in subset])
    kmeans = KMeans(n_clusters=k, random_state=0)
    labels = kmeans.fit_predict(data_array)
    centers = kmeans.cluster_centers_

    # Store centroids
    for center in centers:
        centroid_rows.append({"class": cls, "x": center[0], "y": center[1], "z": ""})

    # Plot clustered data with centroids
    plt.figure(figsize=(40, 40))
    plt.scatter(
        data_array[:, 0],
        data_array[:, 1],
        c=labels,
        cmap="tab10",
        alpha=confidence_vals,
    )
    plt.scatter(
        centers[:, 0],
        centers[:, 1],
        color="black",
        marker="x",
        s=100,
        label="Centroids",
    )
    plt.title(f"K-Means Clustering for Class: {cls} (k={k})")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.legend()
    plt.grid(True)
    plt.show()

# Save all centroids to CSV
with open("centroids_output.csv", "w", newline="") as csvfile:
    fieldnames = ["class", "x", "y", "z"]
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
    writer.writeheader()
    for row in centroid_rows:
        writer.writerow(row)

print("\nCentroids saved to 'centroids_output.csv'")
