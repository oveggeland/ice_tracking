import numpy as np
import os
import glob
import cv2

def read_projection_file(file_path):
    """
    Reads a binary projection file and returns the data as numpy arrays.
    
    Args:
        file_path (str): Path to the binary file to read.

    Returns:
        tuple: A tuple containing:
            - r_world (3, N): The world coordinates matrix (3 x N).
            - r_cam (3, N): The camera coordinates matrix (3 x N).
            - uv (2, N): The 2D projection matrix (2 x N).
    """
    with open(file_path, 'rb') as f:
        # Read the number of points
        num_points = np.fromfile(f, dtype=np.int32, count=1)[0]
        
        # Read r_world (3 x N)
        r_world = np.fromfile(f, dtype=np.float32, count=3 * num_points).reshape(3, num_points)
        
        # Read r_cam (3 x N)
        r_cam = np.fromfile(f, dtype=np.float32, count=3 * num_points).reshape(3, num_points)
        
        # Read uv (2 x N)
        uv = np.fromfile(f, dtype=np.float32, count=2 * num_points).reshape(2, num_points)
    
    return r_world, r_cam, uv


# Example usage
if __name__ == "__main__":

    id = "1725811658.075594"
    # Read projection data
    r_world, r_cam, uv = read_projection_file("/home/oskar/icetrack/output/test/dataset/projections/"+ id + ".bin")
    uv = uv.T.astype(int)  # Transpose and convert to integers

    # Read image
    img_raw = cv2.imread("/home/oskar/icetrack/output/test/dataset/images/"+ id +".png")
    img = img_raw.copy()
    # Iterate over points and draw circles
    for i in range(uv.shape[0]):
        cv2.circle(img, (uv[i, 1], uv[i, 0]), 3, (0, 255, 0), -1) 

    # Display the image with circles
    cv2.imshow("Raw", img_raw)
    cv2.imshow("Projected", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()