package hr.ibelfinger;//
// Copyright 2012 Google
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

import com.google.ortools.constraintsolver.Assignment;
import com.google.ortools.constraintsolver.FirstSolutionStrategy;
import com.google.ortools.constraintsolver.NodeEvaluator2;
import com.google.ortools.constraintsolver.RoutingDimension;
import com.google.ortools.constraintsolver.RoutingModel;
import com.google.ortools.constraintsolver.RoutingSearchParameters;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

class Bla {
    static {
        System.loadLibrary("jniortools");
    }

    public static void main(String[] args) throws Exception {
        main();
    }

    //###########################
    //# Problem Data Definition #
    //###########################
    private static Map<String, Object> create_data_model() {
        Map<String, Object> data = new HashMap<>();
        //# Array of distances between locations.
        int [][] distances = new int[][]{
                {0, 548, 776, 696, 582, 274, 502, 194, 308, 194, 536, 502, 388, 354, 468, 776, 662},
                {548, 0, 684, 308, 194, 502, 730, 354, 696, 742, 1084, 594, 480, 674, 1016, 868, 1210},
                {776, 684, 0, 992, 878, 502, 274, 810, 468, 742, 400, 1278, 1164, 1130, 788, 1552, 754},
                {696, 308, 992, 0, 114, 650, 878, 502, 844, 890, 1232, 514, 628, 822, 1164, 560, 1358},
                {582, 194, 878, 114, 0, 536, 764, 388, 730, 776, 1118, 400, 514, 708, 1050, 674, 1244},
                {274, 502, 502, 650, 536, 0, 228, 308, 194, 240, 582, 776, 662, 628, 514, 1050, 708},
                {502, 730, 274, 878, 764, 228, 0, 536, 194, 468, 354, 1004, 890, 856, 514, 1278, 480},
                {194, 354, 810, 502, 388, 308, 536, 0, 342, 388, 730, 468, 354, 320, 662, 742, 856},
                {308, 696, 468, 844, 730, 194, 194, 342, 0, 274, 388, 810, 696, 662, 320, 1084, 514},
                {194, 742, 742, 890, 776, 240, 468, 388, 274, 0, 342, 536, 422, 388, 274, 810, 468},
                {536, 1084, 400, 1232, 1118, 582, 354, 730, 388, 342, 0, 878, 764, 730, 388, 1152, 354},
                {502, 594, 1278, 514, 400, 776, 1004, 468, 810, 536, 878, 0, 114, 308, 650, 274, 844},
                {388, 480, 1164, 628, 514, 662, 890, 354, 696, 422, 764, 114, 0, 194, 536, 388, 730},
                {354, 674, 1130, 822, 708, 628, 856, 320, 662, 388, 730, 308, 194, 0, 342, 422, 536},
                {468, 1016, 788, 1164, 1050, 514, 514, 662, 320, 274, 388, 650, 536, 342, 0, 764, 194},
                {776, 868, 1552, 560, 674, 1050, 1278, 742, 1084, 810, 1152, 274, 388, 422, 764, 0, 798},
                {662, 1210, 754, 1358, 1244, 708, 480, 856, 514, 468, 354, 844, 730, 536, 194, 798, 0}
            };
        data.put("distances", distances);
        data.put("num_locations", distances.length);
        data.put("num_vehicles", 4);
        data.put("depot", 0);
        //"""Creates the data for the example."""
        return data;
    }

    //#######################
    //# Problem Constraints #
    //#######################
    private static NodeEvaluator2 create_distance_callback(Map<String, Object> data) {
        //"""Creates callback to return distance between points."""
        int [][] distances = (int [][]) data.get("distances");

        return new NodeEvaluator2(){
            @Override
            public long run(int firstIndex, int secondIndex) {
                return distances[firstIndex][secondIndex];
            }
        };
    }

    private static void add_distance_dimension(RoutingModel routing, NodeEvaluator2 distance_callback) {
        //"""Add Global Span constraint"""
        String distance = "Distance";
        int maximum_distance = 3000; // Maximum distance per vehicle.
        routing.addDimension(
                distance_callback,
                0,  // null slack
                maximum_distance,
                true,  // start cumul to zero
                distance);
        final RoutingDimension distance_dimension = routing.getDimensionOrDie(distance);
        // Try to minimize the max distance among vehicles.
        distance_dimension.setGlobalSpanCostCoefficient(100);
    }
    private static void main(){
        //"""Entry point of the program"""
        //# Instantiate the data problem.
        final Map<String, Object> data = create_data_model();
        RoutingModel routing = new RoutingModel(
                (int) data.get("num_locations"),
                (int) data.get("num_vehicles"),
                (int)data.get("depot"));
        // Create Routing Model
        //# Define weight of each edge
        routing.setArcCostEvaluatorOfAllVehicles(create_distance_callback(data));
        add_distance_dimension(routing, create_distance_callback(data));
        //# Setting first solution heuristic (cheapest addition).
        RoutingSearchParameters search_parameters =
                RoutingSearchParameters.newBuilder()
                        .mergeFrom(RoutingModel.defaultSearchParameters())
                        .setFirstSolutionStrategy(FirstSolutionStrategy.Value.PATH_CHEAPEST_ARC)
                        .build();
        //# Solve the problem.
        final Assignment solution = routing.solveWithParameters(search_parameters);

        print_solution(data, routing, solution);

    }
    //###########
    //# Printer #
    //###########
    public static void print_solution(Map<String, Object> data, RoutingModel routing, Assignment assignment) {
        //"""Print routes on console."""
        int total_distance = 0;
        final int num_vehicles = (int) data.get("num_vehicles");
        for (int vehicle_id = 0; vehicle_id < num_vehicles; vehicle_id++) {
            long index = routing.start(vehicle_id);
            String plan_output = "Route for vehicle " + vehicle_id + ":\n";
            int route_dist = 0;
            while (!routing.isEnd(index)) {
                final int node_index = routing.indexToNode(index);
                final int next_node_index = routing.indexToNode(
                        assignment.value(routing.nextVar(index)));
                route_dist += routing.getArcCostForVehicle(node_index, next_node_index, vehicle_id);
                plan_output += " " + node_index + " ->";
                index = assignment.value(routing.nextVar(index));

            }

            plan_output += " "  + routing.indexToNode(index) + "\n";
            plan_output += "Distance of route: " + route_dist + "m\n";
            System.out.println(plan_output);
            total_distance += route_dist;
            System.out.println("Total distance of all routes: " + total_distance + "m");

        }
    }
}
