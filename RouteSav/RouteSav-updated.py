import os
import sys

import networkx as nx
import plotly.graph_objects as go
import osmnx as ox
from PyQt5 import QtWidgets, QtWebEngineWidgets, QtCore
from PyQt5.QtWidgets import QLabel, QComboBox
from plotly import offline
import heapq


def dijkstra_shortest_path(graph, source, target):
    distances = {node: float('inf') for node in graph}
    distances[source] = 0
    previous_nodes = {node: None for node in graph}

    priority_queue = [(0, source)]
    while priority_queue:
        current_distance, current_node = heapq.heappop(priority_queue)

        if current_node == target:
            break

        if current_distance > distances[current_node]:
            continue

        for neighbor, weight in graph[current_node].items():
            distance = current_distance + weight[0]['length']
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                previous_nodes[neighbor] = current_node
                heapq.heappush(priority_queue, (distance, neighbor))

    if distances[target] == float('inf'):
        return None  # No path found

    path = []
    current_node = target
    while current_node is not None:
        path.append(current_node)
        current_node = previous_nodes[current_node]
    path.reverse()

    return path


##### Interface to OSMNX
def generating_path(origin_point, target_point, perimeter):
    # Using the cache accelerates processing for a large map
    # ox.config(log_console=True, use_cache=True, cache_folder='/cache')

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
    # Specify the full path to the HTML file
    if os.path.exists(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'preprocessed_graph.graphml')):
        # Load the pre-processed graph
        roadgraph = ox.load_graphml('preprocessed_graph.graphml')
    else:
        # Create the path/road network graph via setting the perimeters
        roadgraph = ox.graph_from_bbox(north + perimeter, south - perimeter, east + perimeter, west - perimeter,
                                       network_type=mode, simplify=False)
        ox.save_graphml(roadgraph, 'preprocessed_graph.graphml')

    # Get the nearest node in the OSMNX graph for the origin point
    origin_node = ox.distance.nearest_nodes(roadgraph, origin_point[1], origin_point[0])

    # Get the nearest node in the OSMNX graph for the target point
    target_node = ox.distance.nearest_nodes(roadgraph, target_point[1], target_point[0])

    # Get the optimal path via dijkstra
    # route = nx.shortest_path(roadgraph, origin_node, target_node, weight='length', method='dijkstra')
    route = dijkstra_shortest_path(roadgraph, origin_node, target_node)
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
def plot_map(origin_point, target_point, long, lat):
    print(origin_point)
    print(target_point)
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

    # Save the figure as an HTML file
    offline.plot(fig, filename='plot.html', auto_open=False)


##### MAIN
class Window(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.destination_dropdown = None
        self.source_dropdown = None
        self.initWindow()

    def initWindow(self):
        self.setWindowTitle(self.tr("MAP PROJECT"))
        self.setFixedSize(1500, 800)
        self.buttonUI()
        ox.config(use_cache=True, log_console=True)

    def buttonUI(self):
        # Create the "Source" title label
        source_label = QLabel("Starting Point", self)

        # Create the source dropdown
        self.source_dropdown = QComboBox(self)
        self.source_dropdown.addItem("Changi Airport Terminal 3", [1.355819113734586, 103.98637764883509])
        self.source_dropdown.addItem("ibis budget Singapore Pearl", [1.3117510023367127, 103.87940230507937])
        self.source_dropdown.addItem("Min Wah Hotel", [1.312324970031862, 103.8824107783044])
        self.source_dropdown.addItem("Amrise Hotel", [1.3139710326135319, 103.87786884865685])

        # Create the "Destination" title label
        destination_label = QLabel("Destination", self)

        # Create the destination dropdown
        self.destination_dropdown = QComboBox(self)
        self.destination_dropdown.addItem("Changi Airport Terminal 3", [1.355819113734586, 103.98637764883509])
        self.destination_dropdown.addItem("ibis budget Singapore Pearl", [1.3117510023367127, 103.87940230507937])
        self.destination_dropdown.addItem("Min Wah Hotel", [1.312324970031862, 103.8824107783044])
        self.destination_dropdown.addItem("Amrise Hotel", [1.3139710326135319, 103.87786884865685])
        findPathButton = QtWidgets.QPushButton(self.tr("Find path"))
        findPathButton.setFixedSize(120, 50)

        self.view = QtWebEngineWidgets.QWebEngineView()
        self.view.setContentsMargins(25, 25, 25, 25)

        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        lay = QtWidgets.QHBoxLayout(central_widget)

        button_container = QtWidgets.QWidget()
        vlay = QtWidgets.QVBoxLayout(button_container)
        vlay.addStretch()
        vlay.addWidget(source_label)
        vlay.addWidget(self.source_dropdown)
        vlay.addWidget(destination_label)
        vlay.addWidget(self.destination_dropdown)
        hlay = QtWidgets.QHBoxLayout()
        hlay.addWidget(findPathButton)
        vlay.addLayout(hlay)
        vlay.addStretch()
        lay.addWidget(button_container)
        lay.addWidget(self.view, stretch=1)

        # Connect the findPathButton to rout_path function
        findPathButton.clicked.connect(self.route_path)

    def display(self, filename):
        # Get the current directory
        current_directory = os.path.dirname(os.path.abspath(__file__))
        # Specify the full path to the HTML file
        html_file = os.path.join(current_directory, filename)
        self.view.load(QtCore.QUrl.fromLocalFile(html_file))

    def route_path(self):
        self.display('loading.html')
        source = self.source_dropdown.itemData(self.source_dropdown.currentIndex())
        destination = self.destination_dropdown.itemData(self.destination_dropdown.currentIndex())
        # Set the origin and target geocoordinate from which the paths are calculated
        origin_point = (source[0], source[1])
        target_point = (destination[0], destination[1])
        # Create the lists for storing the paths
        long = []
        lat = []

        # Perimeter is the scope of the road network around a geocoordinate
        perimeter = 0.10
        x, y = generating_path(origin_point, target_point, perimeter)
        # Append the paths
        long.append(x)
        lat.append(y)

        plot_map(origin_point, target_point, long, lat)
        self.display('plot.html')


if __name__ == "__main__":
    App = QtWidgets.QApplication(sys.argv)
    window = Window()
    window.show()
    sys.exit(App.exec())
