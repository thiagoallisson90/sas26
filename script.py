from sklearn.cluster import KMeans
import numpy as np  
import pandas as pd
import os

def generate_coords(num_points, axis, seed=42):
    """Generate random coordinates."""
    np.random.seed(seed)
    coords = np.random.uniform(0, axis, (num_points, 2))
    np.random.seed(None)

    return coords

def get_dir():
    return os.getcwd()

def gen_ncoords(n=200):
    """Generate n random coordinates within a 7000x7000 area."""
    coords = generate_coords(n, 7000)
    
    filename = f'{get_dir()}/{n}/{n}sms.csv'
    pd.DataFrame(coords).to_csv(filename, index=False, header=False)
    return coords

def cluster_ncoords(coords, n_clusters=1):
    """Cluster n random coordinates into n_clusters using KMeans."""
    n = len(coords)
    kmeans = KMeans(n_clusters=n_clusters, random_state=42)
    kmeans.fit(coords)
    
    clustered_coords = kmeans.cluster_centers_
    filename = f'{get_dir()}/{n}/{n_clusters}gws.csv'
    pd.DataFrame(clustered_coords).to_csv(filename, index=False, header=False)

if __name__ == '__main__':
    for n in [200, 400, 600, 800, 1000]:
        coords = gen_ncoords(n)

        for k in range(1, 29):
            cluster_ncoords(coords, k)
