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
import com.google.ortools.constraintsolver.IntExpr;
import com.google.ortools.constraintsolver.NodeEvaluator2;
import com.google.ortools.constraintsolver.RoutingDimension;
import com.google.ortools.constraintsolver.RoutingModel;
import com.google.ortools.constraintsolver.FirstSolutionStrategy;
import com.google.ortools.constraintsolver.RoutingSearchParameters;
import com.google.ortools.constraintsolver.Solver;
import hr.ibelfinger.evaluators.CapacityEvaluator;
import hr.ibelfinger.evaluators.StraightLineEvaluator;

class Tsp {
    private static final int CAPACITY = 3;


    static {
        System.loadLibrary("jniortools");
    }

    public static void main(String[] args) {
        final List<Location> locations = LocationConfig.locations();

        final int numberOfVehicles = locations.stream()
                .max(Comparator.comparing(Location::getBookingId)).get().getBookingId() * 2;
        final int depotIndex = 0;

        RoutingModel model = new RoutingModel(locations.size(), numberOfVehicles, depotIndex);

        model.setArcCostEvaluatorOfAllVehicles(new StraightLineEvaluator(locations));
        addDistanceDimension(model, new StraightLineEvaluator(locations));
        addCapacityDimension(model, locations);

        markPickupsAndDropoffs(model, locations);
        markBookingsThatShouldnBeFirstInRoutes(model, locations);

        executeAndPrint(locations, numberOfVehicles, model, FirstSolutionStrategy.Value.GLOBAL_CHEAPEST_ARC);
    }

    private static void addDistanceDimension(RoutingModel routingModel, NodeEvaluator2 distanceCallback){
        String distance = "distance";
        int maximum_distance = Integer.MAX_VALUE;  // Maximum distance per vehicle.
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

    private static void addCapacityDimension(RoutingModel routingModel, List<Location> locations){
        //"""Adds capacity constraint"""
        String capacity = "capacity";

        routingModel.addDimension(
                new CapacityEvaluator(locations),
                0, // null capacity slack
                CAPACITY, // vehicle maximum capacities
                false, // IDK
                capacity);
    }

    private static void executeAndPrint(List<Location> locations, int numberOfVehicles, RoutingModel model,
                                        FirstSolutionStrategy.Value firstSolutionStrategyValue) {
        long startMilis = System.currentTimeMillis();
        RoutingSearchParameters search_parameters =
                RoutingSearchParameters.newBuilder()
                        .mergeFrom(RoutingModel.defaultSearchParameters())
                        .setFirstSolutionStrategy(firstSolutionStrategyValue)
                        .build();

        Assignment solution = model.solveWithParameters(search_parameters);
        if(solution != null) {
            ResultPrinter.printSolution(numberOfVehicles, model, solution, locations);
            System.out.println(firstSolutionStrategyValue.toString() + " GAVE RESULT");
            long endMilis = System.currentTimeMillis();
            System.out.println("Total exec time: " + (endMilis - startMilis) + " milis");
        } else {
            System.out.println(firstSolutionStrategyValue.toString() + " couldnt give result");
        }

    }

    private static void markBookingsThatShouldnBeFirstInRoutes(RoutingModel model, List<Location> locations) {
        for(int i = 0; i < locations.size(); i++) {
            final Location location = locations.get(i);
            if(location.isCnf()) {
                long pickupNodeIndex = model.nodeToIndex(i);

                model.solver()
                                .addConstraint(model.solver().makeGreater(
                        model.cumulVar(pickupNodeIndex, "distance"), 0));
            }
        }
    }

    private static void markPickupsAndDropoffs(RoutingModel model, List<Location> locations) {
        for(int i = 0; i< locations.size(); i++) {
            Location location = locations.get(i);
            if(location.getLocationType() == Location.LocationType.PICKUP) {
                int pickupLocationIndex = i;
                Location pickupLocation = location;

                Integer dropoffLocationIndex = LocationUtils.findLocationIndex(locations, pickupLocation.getBookingId(), Location.LocationType.DROPOFF);
                if(dropoffLocationIndex == null) {
                    throw new RuntimeException("Booking " + pickupLocation.getBookingId() + " has pickup but no dropoff");
                }

                markAsPickupDropoff(model, pickupLocationIndex, dropoffLocationIndex, locations);

            }
        }
    }

    private static void markAsPickupDropoff(RoutingModel model, int pickupArrayIndex, int dropoffArrayIndex, List<Location> locations) {
        long pickupNodeIndex = model.nodeToIndex(pickupArrayIndex);
        long deliveryNodeIndex = model.nodeToIndex(dropoffArrayIndex);
        final Solver solver = model.solver();

        model.AddPickupAndDelivery(pickupArrayIndex, dropoffArrayIndex);

        solver.addConstraint(solver.makeLessOrEqual(
                model.cumulVar(pickupNodeIndex, "distance"),
                model.cumulVar(deliveryNodeIndex, "distance")));

        solver.addConstraint(solver.makeEquality(
                model.vehicleVar(pickupNodeIndex),
                model.vehicleVar(deliveryNodeIndex)));

        //for every booking, new trip length shouldn't be larger than original * 1.5
        final long bookingDistance = LocationUtils.getLocationDistance(locations.get(pickupArrayIndex), locations.get(dropoffArrayIndex));
        final double maxAllowedBookingDistance = bookingDistance * 1.5;

        final IntExpr pickupToDropoffDistance = solver.makeDifference(
                model.cumulVar(deliveryNodeIndex, "distance"),
                model.cumulVar(pickupNodeIndex, "distance"));
        solver.addConstraint(
                solver.makeLessOrEqual(pickupToDropoffDistance, (int)maxAllowedBookingDistance));
    }
}
