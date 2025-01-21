import pandas as pd
import json
from scipy.interpolate import interp1d
from datetime import datetime

import os
import yaml

if __name__ == "__main__":
    with open("/home/oskar/icetrack/src/icetrack/config/info.yaml", "r") as file:
        config = yaml.safe_load(file)

        output_dir = config["outpath"]
    
        # Input and output file paths
        nav_file = os.path.join(output_dir, "navigation/ins.csv")
        stats_file = os.path.join(output_dir, "stats/stats.csv")
        output_file = os.path.join(output_dir, "stats/track_stats.geojson")
        

    # Read CSVs into dataframes
    nav_df = pd.read_csv(nav_file)
    stats_df = pd.read_csv(stats_file)

    # Convert timestamps to float for interpolation
    nav_df['ts'] = nav_df['ts'].astype(float)
    stats_df['ts'] = stats_df['ts'].astype(float)

    # Interpolate navigation data to match statistics timestamps
    interp_funcs = {}
    for column in ['x', 'y']:
        interp_funcs[column] = interp1d(nav_df['ts'], nav_df[column], kind='linear', bounds_error=False, fill_value='extrapolate')

    # Add interpolated position to the statistics dataframe
    stats_df['x'] = interp_funcs['x'](stats_df['ts'])
    stats_df['y'] = interp_funcs['y'](stats_df['ts'])

    # Convert timestamps to ISO datetime strings
    stats_df['datetime'] = pd.to_datetime(stats_df['ts'], unit='s').dt.strftime('%Y-%m-%dT%H:%M:%S.%fZ')

    # Prepare GeoJSON features
    features = []
    for _, row in stats_df.iterrows():
        feature = {
            "type": "Feature",
            "geometry": {
                "type": "Point",
                "coordinates": [row['y'], row['x']]
            },
            "properties": {
                "datetime": row['datetime'],
                "count": row['count'],
                "z_mean": row['z_mean'],
                "z_var": row['z_var'],
                "i_mean": row['i_mean'],
                "i_var": row['i_var'],
                "z_exp_mean": row['z_exp_mean'],
                "z_exp_var": row['z_exp_var']
            }
        }
        features.append(feature)

    # Create GeoJSON structure
    geojson = {
        "type": "FeatureCollection",
        "features": features
    }

    # Save to a GeoJSON file
    with open(output_file, 'w') as f:
        json.dump(geojson, f, indent=4)

    print(f"GeoJSON file saved to {output_file}")