import numpy as np
import open3d as o3d
import scipy.stats
import matplotlib.pyplot as plt

import os

def analyseCorrelation(pcd):
    elevation = -pcd.point.positions[:, 2].numpy().flatten()
    intensity = pcd.point.intensities.numpy().flatten()
    distance = pcd.point.distances.numpy().flatten()
    
    corrcoeff = np.corrcoef(np.stack((elevation, intensity, distance)))
    print(corrcoeff)
    
    plt.figure(f"Elevation vs Distance ({corrcoeff[0, 2]})")
    plt.scatter(elevation, distance, alpha=0.05)
    plt.xlabel("Elevation")
    plt.ylabel("Distance")

    plt.figure(f"Elevation vs Intensity ({corrcoeff[0, 1]})")
    plt.scatter(elevation, distance, alpha=0.05)
    plt.xlabel("Elevation")
    plt.ylabel("Intensity")
    
    plt.figure(f"Distance vs Intensity ({corrcoeff[2, 1]})")
    plt.scatter(distance, intensity, alpha=0.05)
    plt.xlabel("Distance")
    plt.ylabel("Intensity")

    plt.show()
    



if __name__ == "__main__":
    out_path = "/home/oskar/icetrack/output/long3/"
    cloud_path = os.path.join(out_path, "clouds")
    hist_path = os.path.join(out_path, "hist")
    os.makedirs(hist_path, exist_ok=True)
    
    cloud_names = sorted(os.listdir(cloud_path))
    n_clouds = len(cloud_names)
    
    ts = np.empty(n_clouds, dtype=float)
    count = np.empty_like(ts)
    
    # From negative values
    freeboard = np.empty_like(ts)
    
    # Analyse elevation (as gamma distribution)
    k = np.empty_like(ts)
    theta = np.empty_like(ts)
    
    # Deformation
    mean_deformation = np.empty_like(ts)
    min_deformation = np.empty_like(ts)
    max_deformation = np.empty_like(ts)
    cum_deformation = np.empty_like(ts)

    for i, name in enumerate(cloud_names):
        # File         
        cloudfile = os.path.join(cloud_path, name)
        label, ext = os.path.splitext(name)
        ts[i] = float(label)
        
        # Extract features
        pcd = o3d.t.io.read_point_cloud(cloudfile)
        count[i] = pcd.point.positions.shape[0]
        
        # analyseCorrelation(pcd)

        # Analyse deformation
        deformation = pcd.point.deformation.flatten().numpy()
        mean_deformation[i] = deformation.mean()
        min_deformation[i] = deformation.min()
        max_deformation[i] = deformation.max()
        cum_deformation[i] = deformation.sum()
        
        # Analyse elevation
        elevation = pcd.point.positions[0:, 2].flatten().numpy()
        positive = elevation > 0
        elev_pos = elevation[positive]
        elev_neg = elevation[~positive]

        ## Make histogram where positive and negative values are in different colors
        bin_width = 0.05

        plt.figure("hist")
        plt.hist(elev_pos, range=(0, 3), bins=int(3/bin_width), color='blue', alpha=0.7, label='Positive Elevation')
        plt.hist(elev_neg, range=(-1, 0), bins=int(1/bin_width), color='red', alpha=0.7, label='Negative Elevation', )
        plt.savefig(os.path.join(hist_path, label+".png"))
        plt.clf()

        # Estimate deformation from positive
        (k[i], _, theta[i]) = scipy.stats.gamma.fit(elev_pos, floc=0)
        
    
    # Gamma distribution analysis
    mean = k*theta
    mode = (k-1)*theta
    var = k*theta*theta

    plt.figure("Count")
    plt.plot(ts, count)
    
    plt.figure("Gamma mean/mode")
    plt.plot(ts, mean, label='mean')
    plt.plot(ts, mode, label='mode')
    plt.legend()
    
    plt.figure("Gamma variance")
    plt.plot(ts, var)
    
    plt.figure("Normalized variance")
    plt.plot(ts, var*count)
    
    plt.figure("Deformation")
    plt.plot(ts, min_deformation)
    plt.plot(ts, max_deformation)
    plt.plot(ts, mean_deformation)
    
    plt.figure("Cumulative deformation")
    plt.plot(ts, cum_deformation)
    
    plt.show()
        #analyseElevation(elevation, out_path, label)