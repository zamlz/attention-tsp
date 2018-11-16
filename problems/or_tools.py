
from math import sqrt
from tqdm import tqdm
from problem_vrp import CVRP
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2


# This lets us convert our dataset from the attention-tsp format
# into a format thats suitable for google OR-tools
def create_data_model(ds):
    """Stores the data for the problem"""
    data = {}
    locations = [tuple(ds['depot'])] + [ tuple(x) for x in list(ds['loc']) ]
    capacities = [ds['capacity']]
    demands = [0.0] + [ capacities[0]*d for d in list(ds['demand']) ]
    
    data["locations"] = [ (l[0], l[1]) for l in locations ]
    data["num_locations"] = len(data["locations"])
    data["num_vehicles"] = 1
    data["depot"] = 0
    data["demands"] = demands
    data["vehicle_capacities"] = data['num_vehicles']*capacities
    return data


def euclidean_distance(p1, p2):
    """Computes the Euclidean distance between two points"""
    return (sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2))
def create_distance_callback(data):
    """Creates callback to return distance between points."""
    _distances = {}
    for from_node in range(data["num_locations"]):
        _distances[from_node] = {}
        for to_node in range(data["num_locations"]):
            if from_node == to_node:
                _distances[from_node][to_node] = 0
            else:
                _distances[from_node][to_node] = (
                        euclidean_distance(data["locations"][from_node],
                                           data["locations"][to_node]))
    def distance_callback(from_node, to_node):
        """Returns the manhattan distance between the two nodes"""
        return _distances[from_node][to_node]
    return distance_callback


def create_demand_callback(data):
    """Creates callback to get demands at each location."""
    def demand_callback(from_node, to_node):
        return data["demands"][from_node]
    return demand_callback


def add_capacity_constraints(routing, data, demand_callback):
    """Adds capacity constraint"""
    capacity = "Capacity"
    routing.AddDimensionWithVehicleCapacity(
        demand_callback,
        0, # null capacity slack?
        data["vehicle_capacities"], # vehicle maximum capacities
        True, # start cumul to zero
        capacity)


def print_solution(data, routing, assignment):
    """Print routes on console"""
    total_dist = 0
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        route_dist = 0
        route_load = 0
        while not routing.IsEnd(index):
            node_index = routing.IndexToNode(index)
            next_node_index = routing.IndexToNode(assignment.Value(
                    routing.NextVar(index)))
            route_dist += euclidean_distance(
                    data["locations"][node_index],
                    data["locations"][next_node_index])
            route_load += data["demands"][node_index]
            plan_output += ' {0} Load({1}) -> '.format(node_index, route_load)
            index = assignment.Value(routing.NextVar(index))

        node_index = routing.IndexToNode(index)
        total_dist += route_dist
        plan_output += ' {0} Load({1})\n'.format(node_index, route_load)
        plan_output += 'Distance of the route: {0}m\n'.format(route_dist)
        plan_output += 'Load of the route: {0}\n'.format(route_load)
        print(plan_output)
    print('Total Distance of all routes: {0}m'.format(total_dist))


def main():

    graph_size = 10
    trials=1

    print("Running {} trials on graph size of {}.".format(trials, graph_size))

    # Make our dataset for the capacitated vehicle routing problem
    DS = CVRP.make_dataset(size=graph_size, num_samples=trials, filename=None)

    for i in tqdm(range(trials)):

        # Create data for the or tool program
        data = create_data_model(DS[i])

        # Create routing model
        routing = pywrapcp.RoutingModel(
                data["num_locations"],
                data["num_vehicles"],
                data["depot"])

        # Define weight of each edge
        distance_callback = create_distance_callback(data)
        routing.SetArcCostEvaluatorOfAllVehicles(distance_callback)

        # Add Capacity constraint
        demand_callback = create_demand_callback(data)
        add_capacity_constraints(routing, data, demand_callback)

        # Setting first solution heuristic (cheapest addition).
        search_parameters =pywrapcp.RoutingModel.DefaultSearchParameters()
        search_parameters.first_solution_strategy = (
                routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

        # Solve the problem
        assignment = routing.SolveWithParameters(search_parameters)
        if assignment:
            print_solution(data, routing, assignment)

if __name__ == '__main__':
    main()
