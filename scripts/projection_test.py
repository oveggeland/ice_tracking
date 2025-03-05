import cv2
import pandas as pd
import numpy as np

# Load the image (adjust the path to your image)
image_path = '/home/oskar/icetrack/workspace/long3/david/image_frames/images/raw/1725811522075.png'
image = cv2.imread(image_path)

# Read the CSV file (adjust the path to your CSV file)
csv_path = '/home/oskar/icetrack/workspace/long3/david/image_frames/projections/1725811522075.csv'
data = pd.read_csv(csv_path)

# Normalize the elevation values to a 0-255 range using vectorized operations
elevation_min = data['elevation'].min()
elevation_max = data['elevation'].max()

# Vectorized normalization of elevation
normalized_elevations = np.interp(data['elevation'], [elevation_min, elevation_max], [0, 255]).astype(np.uint8)

# Create the color map for all points at once (vectorized)
colors = cv2.applyColorMap(normalized_elevations[:, None], cv2.COLORMAP_JET).squeeze().astype(int)

u_coords = data['u'].astype(int).values
v_coords = data['v'].astype(int).values

image[v_coords, u_coords] = colors

# Save or display the result
cv2.imwrite('output_image_colored_optimized.png', image)  # Save the modified image
cv2.imshow('Image with Colored Circles', image)  # Display the image
cv2.waitKey(0)