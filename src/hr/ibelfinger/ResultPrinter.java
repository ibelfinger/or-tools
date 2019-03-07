package hr.ibelfinger;

import com.google.ortools.constraintsolver.Assignment;
import com.google.ortools.constraintsolver.RoutingModel;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;

public class ResultPrinter {
    //###########
    //# Printer #
    //###########
    public static void printSolution(int numberOfVehicles, RoutingModel routing, Assignment assignment, List<Location> locations) {
        //"""Print routes on console."""

        Map<Integer, Integer> bookingNumberMap = new HashMap<>();

        int total_distance = 0;
        for (int vehicle_id = 0; vehicle_id < numberOfVehicles; vehicle_id++) {
            long index = routing.start(vehicle_id);
            String plan_output = "Route for vehicle " + vehicle_id + ":\n";

            List<Location> route = new ArrayList<>();

            int route_dist = 0;
            int routeLength = 0;
            while (!routing.isEnd(index)) {
                final int node_index = routing.indexToNode(index);
                final int next_node_index = routing.indexToNode(
                        assignment.value(routing.nextVar(index)));
                int lengthBefore = route_dist;
                route_dist += routing.getArcCostForVehicle(node_index, next_node_index, vehicle_id);
                final Location location = locations.get(node_index);

                index = assignment.value(routing.nextVar(index));
                final long capacity = assignment.value(routing.cumulVar(index, "capacity"));

                if(location.getLocationType() != Location.LocationType.DEPOT) {
                    route.add(location);
                    plan_output += " " + location + " (capacity: "+capacity + ", distance: " + lengthBefore + ") ->";
                    routeLength++;
                }


            }

            bookingNumberMap.put(routeLength, bookingNumberMap.getOrDefault(routeLength, 0) + 1);
            final Map<Integer, Double> detours = getDetours(route);

            for(Map.Entry<Integer, Double> entry : detours.entrySet()) {
                System.out.println("BookingId: " + entry.getKey() + " , detour ratio: " + entry.getValue());
            }
            plan_output += " "  + routing.indexToNode(index) + "\n";
            plan_output += "Distance of route: " + route_dist + "m\n";
            System.out.println(plan_output);
            total_distance += route_dist;



        }
        System.out.println("Route sizes: " + bookingNumberMap);
        System.out.println("Total distance of all routes: " + total_distance + "m");


    }

    private static Map<Integer, Double> getDetours(List<Location> route) {
        Set<Integer> bookingIds = route.stream().map(Location::getBookingId).collect(Collectors.toSet());

        Map<Integer, Double> detourRatios = new HashMap<>();

        for(Integer bookingId : bookingIds) {
            Integer pickupLocationIndex = LocationUtils.findLocationIndex(route, bookingId, Location.LocationType.PICKUP);
            Location pickupLocation = route.get(pickupLocationIndex);

            Integer dropoffLocationIndex = LocationUtils.findLocationIndex(route, bookingId, Location.LocationType.DROPOFF);
            Location dropoffLocation = route.get(dropoffLocationIndex);

            final long originalBookingDistance = LocationUtils.getLocationDistance(pickupLocation, dropoffLocation);
            long bookingDistanceWithDetour = 0;
            for(int i = pickupLocationIndex; i <= dropoffLocationIndex - 1; i ++) {
                Location firstLocation = route.get(i);
                Location secondLocation = route.get(i + 1);

                bookingDistanceWithDetour += LocationUtils.getLocationDistance(firstLocation, secondLocation);

            }

            detourRatios.put(bookingId, ((double) bookingDistanceWithDetour / originalBookingDistance));
        }

        return detourRatios;

    }
}
