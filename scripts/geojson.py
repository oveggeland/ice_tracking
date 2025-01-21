"""
Read from ins.csv file with the following format:

ts,x,y,z,vx,vy,vz,roll,pitch,yaw,bax,bay,baz,bgx,bgy,bgz,Lx,Ly,Lz
1725782910.000452,2610313.282476,20481922.920869,-16.880331,0.098737,0.076669,0.000000,-0.003600,-0.509004,0.000937,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,8.225920,-0.053070,14.740319

Produce a geojson file with datetime, x and y coordinates. 
"""
    
import os
import yaml

import csv
import json
from datetime import datetime


def read_csv_to_geojson(csv_file, geojson_file):
    # Initialize the GeoJSON structure
    geojson = {
        "type": "FeatureCollection",
        "features": []
    }

    with open(csv_file, "r") as file:
        reader = csv.DictReader(file)

        for row in reader:
            # Convert timestamp to ISO 8601 datetime format
            ts = float(row["ts"])
            dt = datetime.utcfromtimestamp(ts).isoformat() + "Z"

            # Extract x and y coordinates
            x = float(row["x"])
            y = float(row["y"])

            # Create a GeoJSON feature
            feature = {
                "type": "Feature",
                "properties": {
                    "datetime": dt
                },
                "geometry": {
                    "type": "Point",
                    "coordinates": [y, x]
                }
            }

            # Append the feature to the GeoJSON structure
            geojson["features"].append(feature)

    # Write the GeoJSON to a file
    with open(geojson_file, "w") as geojson_out:
        json.dump(geojson, geojson_out, indent=2)


if __name__ == "__main__":
    with open("/home/oskar/icetrack/src/icetrack/config/info.yaml", "r") as file:
        config = yaml.safe_load(file)

        output_dir = config["outpath"]
    
        # Input and output file paths
        input_csv_file = os.path.join(output_dir, "navigation/ins.csv")
        output_geojson_file = os.path.join(output_dir, "navigation/track_estimate.geojson")

        # Convert the CSV to GeoJSON
        read_csv_to_geojson(input_csv_file, output_geojson_file)
        print(f"GeoJSON file '{output_geojson_file}' has been created.")
