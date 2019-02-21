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
import java.util.*;

import com.google.ortools.constraintsolver.Assignment;
import com.google.ortools.constraintsolver.NodeEvaluator2;
import com.google.ortools.constraintsolver.RoutingDimension;
import com.google.ortools.constraintsolver.RoutingModel;
import com.google.ortools.constraintsolver.FirstSolutionStrategy;
import com.google.ortools.constraintsolver.RoutingSearchParameters;
import com.google.ortools.constraintsolver.Solver;

class Tsp {
    static {
        System.loadLibrary("jniortools");
    }

    private static class Location {
        public final double latitude;
        public final double longitude;
        public final LocationType locationType;

        private Location(LocationType locationType, double latitude, double longitude) {
            this.locationType = locationType;
            this.latitude = latitude;
            this.longitude = longitude;
        }

        public static enum LocationType {
            DEPOT {
                @Override
                public int capacityModifier() {
                    return 0;
                }
            },
            PICKUP {
                @Override
                public int capacityModifier() {
                    return -1;
                }
            },
            DROPOFF {
                @Override
                public int capacityModifier() {
                    return +1;
                }
            };

            public abstract int capacityModifier();
        }
    }

    private static class StraightLine extends NodeEvaluator2 {
        private final List<Location> locations;

        public StraightLine(List<Location> locations) {
            this.locations = locations;
        }

        @Override
        public long run(int firstIndex, int secondIndex) {
            if(firstIndex == 0 || secondIndex == 0) {
                return 0; //distance to and from depot is 0. This imitates that we don't care where car ends up
            }
            Location firstLocation = locations.get(firstIndex);
            Location secondLocation = locations.get(secondIndex);

            return (long) (Math.abs(firstLocation.latitude - secondLocation.latitude)
                    + Math.abs(firstLocation.longitude - secondLocation.longitude) * 100);
        }
    }

    private static class CapacityEvaluator extends NodeEvaluator2 {
        private final List<Location> locations;

        public CapacityEvaluator(List<Location> locations) {
            this.locations = locations;
        }

        @Override
        public long run(int fromIndex, int toIndex) {
            final int capacityModifier = locations.get(fromIndex).locationType.capacityModifier();
            return capacityModifier;
        }
    }



    private static void addDistanceDimension(RoutingModel routingModel, NodeEvaluator2 distanceCallback){
        String distance = "distance";
        int maximum_distance = 3000;  // Maximum distance per vehicle.
        routingModel.addDimension(
                distanceCallback,
                0,  // null slack
                maximum_distance,
                true,  // start cumul to zero
                distance);
        final RoutingDimension distanceDimension = routingModel.getDimensionOrDie(distance);
        // Try to minimize the max distance among vehicles.
        distanceDimension.setGlobalSpanCostCoefficient(100);
    }

    private static void addCapacityDimension(RoutingModel routingModel, List<Location> locations, int numberOfVehicles){
        //"""Adds capacity constraint"""
        String capacity = "capacity";
        long[] capacities = new long[numberOfVehicles];
        for(int i = 0; i < capacities.length; i++) {
            capacities[i] = 2;
        }


        routingModel.addDimensionWithVehicleCapacity(
                new CapacityEvaluator(locations),
                0, // null capacity slack
                capacities, // vehicle maximum capacities
                false, // IDK
                capacity);
    }

    private static void whatIKnow() {
        final List<Location> locations = Arrays.asList(
                new Location(Location.LocationType.DEPOT, 0,0), //we need depot
                new Location(Location.LocationType.PICKUP, 1, 1),
                new Location(Location.LocationType.PICKUP, 2, 2),
                new Location(Location.LocationType.PICKUP, 3,3),
                new Location(Location.LocationType.DROPOFF, 4,4),
                new Location(Location.LocationType.DROPOFF, 11, 11),
                new Location(Location.LocationType.DROPOFF, 10, 10)
        );

        final int numberOfVehicles = 3;
        final int depotIndex = 0;

        RoutingModel model =
                new RoutingModel(locations.size(), numberOfVehicles, depotIndex);

        model.setArcCostEvaluatorOfAllVehicles(new StraightLine(locations));
        addDistanceDimension(model, new StraightLine(locations));
        addCapacityDimension(model, locations, numberOfVehicles);

        markAsPickupDropoff(model, 1, 4);
        markAsPickupDropoff(model, 2, 5);
        markAsPickupDropoff(model, 3, 6);

        RoutingSearchParameters search_parameters =
                RoutingSearchParameters.newBuilder()
                        .mergeFrom(RoutingModel.defaultSearchParameters())
                        .setFirstSolutionStrategy(FirstSolutionStrategy.Value.PATH_CHEAPEST_ARC)
                        .build();

        Assignment solution = model.solveWithParameters(search_parameters);

        print_solution(numberOfVehicles, model, solution);
    }

    private static void markAsPickupDropoff(RoutingModel model, int pickupArrayIndex, int dropoffArrayIndex) {
        long pickupNodeIndex = model.nodeToIndex(pickupArrayIndex);
        long deliveryNodeIndex = model.nodeToIndex(dropoffArrayIndex);
        final Solver solver = model.solver();
//
//        //Notifies that node1 and node2 form a pair of nodes which should belong to the same route.
//        //This is a hit to solver. Constraints should still be there, this just helps the optimization
//        //Arguments: NodeIndex node1, NodeIndex node2
        model.AddPickupAndDelivery(pickupArrayIndex, dropoffArrayIndex);
//
//
        solver.addConstraint(solver.makeLessOrEqual(
                model.cumulVar(pickupNodeIndex, "distance"),
                model.cumulVar(deliveryNodeIndex, "distance")));

        solver.addConstraint(solver.makeEquality(
                model.vehicleVar(pickupNodeIndex),
                model.vehicleVar(deliveryNodeIndex)));
    }

    //###########
    //# Printer #
    //###########
    public static void print_solution(int numberOfVehicles, RoutingModel routing, Assignment assignment) {
        //"""Print routes on console."""
        int total_distance = 0;
        for (int vehicle_id = 0; vehicle_id < numberOfVehicles; vehicle_id++) {
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

        }
        System.out.println("Total distance of all routes: " + total_distance + "m");
    }


    public static void main(String[] args) throws Exception {
        whatIKnow();
    }
}
