import matplotlib.image as mpimg
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from Path_Finder import TaxiwayGraph
from Path_Follower import AircraftSimulation

# Map Layout
vertices = ['Gate 1', 'Gate 2', 'Gate 3', 'Gate 4', 'Gate 5', 'Gate 6', 'Gate 7', 'Gate 8',
            'R10L', 'R10S', 'R28L', 'R28S', 'R5L', 'R5S', 'R23', 
            'CC4', 'CC3', 'CC2', 'CC1', 'BB5', 'BB4', 'ED', 'DB', 'CB', 'BB3', 'AB2', 'AA', 'LK']

edges = [('CC4', 'CC3', 1), ('CC4', 'R10L', 1), ('CC3', 'R10S', 1), ('R10L', 'R10S', 1), ('CC3', 'CC2', 2),
         ('CC2', 'CC1', 2), ('Gate 1', 'CC1', 2), ('CC1', 'CB', 2), ('BB4', 'BB5', 1), ('BB5', 'R5L', 1), 
         ('BB4', 'CB', 3), ('BB4', 'R5S', 1), ('R5L', 'R5S', 1), ('CB', 'BB3', 1), ('CB', 'DB', 1), ('AA', 'R28L', 1),
         ('AA', 'R28S', 1), ('R28L', 'R28S', 1), ('AA', 'AB2', 3), ('Gate 6', 'AB2', 1), ('Gate 7', 'AB2', 1), 
         ('Gate 8', 'R23', 1), ('R23', 'BB3', 5), ('Gate 5', 'DB', 2), ('Gate 2', 'ED', 1), ('Gate 3', 'ED', 1), 
         ('Gate 4', 'ED', 1), ('ED', 'LK', 1), ('LK', 'DB', 1), ('BB3', 'AB2', 3), ('R23', 'AB2', 2)]

pos = {'Gate 1': (1093, 579), 'Gate 2': (1234, 562), 'Gate 3': (1250, 661), 'Gate 4': (1340, 647), 'Gate 5': (1394, 638), 'Gate 6': (1510, 706), 'Gate 7': (1573, 701), 'Gate 8': (1646, 699),
        'CC4': (95, 670), 'CC3': (165, 668), 'CC2': (656, 589), 'CC1': (1090, 524), 'ED': (1304, 613), 'DB': (1393, 564), 'CB': (1320, 481), 'AB2': (1583, 611), 'BB5': (888, 99), 'BB4': (946, 149), 
        'BB3': (1370, 437), 'AA': (1717, 440), 'R10L': (71, 589), 'R10S': (155, 577), 'R28L': (1722, 323), 'R28S': (1652, 338), 'R5L': (927, 50), 'R5S': (989, 105), 'R23': (1734, 742), 'LK': (1355, 567)}

image_raw = mpimg.imread("map.png") # Load the airport map image
image = np.flipud(image_raw)

def main():
    taxiway_graph = TaxiwayGraph(vertices, edges)
    
    start_gate = 'Gate 1'
    end_runway = 'R10L'

    shortest_path_flight, control_points = taxiway_graph.find_shortest_path_flight(start_gate, end_runway ,pos)
    # this makes it sharper
    tension = .8 # Adjust this parameter to control the tension of the spline
    detail_points, derivative_points = taxiway_graph.cardinal_spline(control_points, tension=tension)

    simulation = AircraftSimulation(detail_points)
    simulation.run_simulation()
    simulation.plot_results_animated()

main()

