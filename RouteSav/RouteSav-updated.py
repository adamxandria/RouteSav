import os
import networkx as nx
import plotly.graph_objects as go
import osmnx as ox
import pandas as pd
import geopandas


##### Interface to OSMNX
def generating_path(origin_point, target_point, perimeter):
    # Using the cache accelerates processing for a large map
    ox.config(log_console=True, use_cache=True)

    # Splice the geographical coordinates in long and lat
    origin_lat = origin_point[0]
    origin_long = origin_point[1]

    target_lat = target_point[0]
    target_long = target_point[1]

    # Build the geocoordinate structure of the path's graph

    # If the origin is further from the equator than the target
    if origin_lat > target_lat:
        north = origin_lat
        south = target_lat
    else:
        north = target_lat
        south = origin_lat

    # If the origin is further from the prime meridian than the target
    if origin_long > target_long:
        east = origin_long
        west = target_long
    else:
        east = target_long
        west = origin_long

    # Construct the road graph
    # Modes 'drive'
    mode = 'drive'

    # Create the path/road network graph via setting the perimeters
    roadgraph = ox.graph_from_bbox(north + perimeter, south - perimeter, east + perimeter, west - perimeter,
                                   network_type=mode, simplify=False)


    # Get the nearest node in the OSMNX graph for the origin point
    origin_node = ox.distance.nearest_nodes(roadgraph, origin_point[1], origin_point[0])

    # Get the nearest node in the OSMNX graph for the target point
    target_node = ox.distance.nearest_nodes(roadgraph, target_point[1], target_point[0])

    # Get the optimal path via dijkstra
    route = nx.shortest_path(roadgraph, origin_node, target_node, weight='length', method='dijkstra')

    # Create the arrays for storing the paths
    lat = []
    long = []

    for i in route:
        point = roadgraph.nodes[i]
        long.append(point['x'])
        lat.append(point['y'])

    # Return the paths
    return long, lat


##### Plot the results using mapbox and plotly
def plot_map(origin_point, target_points, long, lat):
    print(origin_point)
    print(target_points)
    print(long)
    print(lat)
    # Create a plotly map and add the origin point to the map
    print("Plotting map...")
    fig = go.Figure(go.Scattermapbox(
        name="Origin",
        mode="markers",
        lon=[origin_point[1]],
        lat=[origin_point[0]],
        marker={'size': 16, 'color': "#333333"},
    )

    )

    # Plot the optimal paths to the map
    print("Generating paths.....")
    for i in range(len(lat)):
        fig.add_trace(go.Scattermapbox(
            name="Path",
            mode="lines",
            lon=long[i],
            lat=lat[i],
            marker={'size': 10},
            showlegend=False,
            line=dict(width=4.5, color='#ff0000'))
        )

    # Plot the target geocoordinates to the map
    print("Generating target...")
    for target_point in target_points:
        fig.add_trace(go.Scattermapbox(
            name="Destination",
            mode="markers",
            showlegend=False,
            lon=[target_point[1]],
            lat=[target_point[0]],
            marker={'size': 16, 'color': '#ff0000'}))

    # Style the map layout
    fig.update_layout(
        mapbox_style="streets",
        mapbox_accesstoken="pk.eyJ1IjoiYWRhbXhhbmRyaWEiLCJhIjoiY2xqanRhbHpkMGFzbDNsbXU5bGxvaG9kcyJ9.0kKSEs9qPLBECjCqbNZ68A",
        legend=dict(yanchor="top", y=1, xanchor="left", x=0.83),  # x 0.9
        title="<span style='font-size: 32px;'><b>RouteSAV</b></span>",
        font_family="Times New Roman",
        font_color="#333333",
        title_font_size=32,
        font_size=18,
        width=1920,
        height=1080,
    )

    # Set the center of the map
    lat_center = 1.3563
    long_center = 103.9865

    # Add the center to the map layout
    fig.update_layout(margin={"r": 0, "t": 0, "l": 0, "b": 0},
                      title=dict(yanchor="top", y=.97, xanchor="left", x=0.03),  # x 0.75
                      mapbox={
                          'center': {'lat': lat_center,
                                     'lon': long_center},
                          'zoom': 12.2}
                      )

    # Save map in output folder
    print("Saving image to output folder...");
    fig.write_image(OS_PATH + '/output/SingaporeMap.jpg', scale=3)

    # Show the map in the web browser
    print("Generating the map in browser...");
    fig.show()


##### MAIN

# Data import path
OS_PATH = os.path.dirname(os.path.realpath('__file__'))
CSV = OS_PATH + '/data/geocoordinates.csv'

# Data Import
df1 = pd.read_csv(CSV)

# Keep only relevant columns
df = df1.loc[:, ("LATITUDE", "LONGITUDE")]

# Create point geometries
geometry = geopandas.points_from_xy(df.LONGITUDE, df.LATITUDE)
geo_df = geopandas.GeoDataFrame(df[['LATITUDE', 'LONGITUDE']], geometry=geometry)

# Format the target geocoordinates from the csv file
target_points = []
for lo, la in zip(df["LONGITUDE"], df["LATITUDE"]):
    print(lo)
    target_points.append((la, lo))

# Set the origin geocoordinate from which the paths are calculated
origin_point = (1.3563, 103.9865)

# Create the lists for storing the paths
long = []
lat = []

i = 0
for target_point in target_points:
    # Perimeter is the scope of the road network around a geocoordinate
    perimeter = 0.10

    # Process the optimal path
    print("Processing Please wait ********************** " + str(i))
    x, y = generating_path(origin_point, target_point, perimeter)

    # Append the paths
    long.append(x)
    lat.append(y)

    i += 1

plot_map(origin_point, target_points, long, lat)
