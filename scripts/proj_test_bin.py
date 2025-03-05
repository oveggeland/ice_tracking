import cv2
import numpy as np
import os

# Load the image (adjust the path to your image)
label = "1725813749077"
MAX_DT = 0.5
image_path = f"/home/oskar/icetrack/workspace/long3/david/image_frames/images/raw/{label}.png"
image = cv2.imread(image_path)

# Read the binary file (adjust the path to your binary file)
binary_path = f"/home/oskar/icetrack/workspace/long3/david/image_frames/projections/{label}.bin"

# Read the number of points from the first 8 bytes (unsigned long)
with open(binary_path, 'rb') as f:
    num_points = np.fromfile(f, dtype=np.uint64, count=1)[0]

# Read the rest of the binary data (5 * num_points float32 values)
data = np.fromfile(binary_path, dtype=np.float32, offset=8)

# Reshape data into (num_points, 5)
data = data.reshape((num_points, 5))

# Extract the columns from the reshaped data
u_coords = data[:, 0].astype(int)  # u coordinates
v_coords = data[:, 1].astype(int)  # v coordinates
elevations = data[:, 2]  # elevation values
dt = data[:, 4]  # dt values

# Filter out points where abs(dt) > 0.1
valid_indices = np.abs(dt) <= MAX_DT

# Apply the filter to the data
u_coords = u_coords[valid_indices]
v_coords = v_coords[valid_indices]
elevations = elevations[valid_indices]

# Normalize the elevation values to a 0-255 range using vectorized operations
elevation_min = elevations.min()
elevation_max = elevations.max()

# Vectorized normalization of elevation
normalized_elevations = np.interp(elevations, [elevation_min, elevation_max], [0, 255]).astype(np.uint8)

# Create the color map for all points at once (vectorized)
colors = cv2.applyColorMap(normalized_elevations[:, None], cv2.COLORMAP_JET).squeeze().astype(int)

# Update the image at the u, v coordinates
image[v_coords, u_coords] = colors

# Save or display the result
cv2.imwrite('output_image_colored_from_binary_np_filtered.png', image)  # Save the modified image
cv2.imshow('Image with Colored Points', image)  # Display the image
cv2.waitKey(0)
cv2.destroyAllWindows()