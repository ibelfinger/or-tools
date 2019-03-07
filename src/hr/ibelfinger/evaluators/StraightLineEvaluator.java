package hr.ibelfinger.evaluators;

import com.google.ortools.constraintsolver.NodeEvaluator2;
import com.google.ortools.constraintsolver.RoutingModel;
import hr.ibelfinger.Location;
import hr.ibelfinger.LocationUtils;

import java.util.List;

public class StraightLineEvaluator extends NodeEvaluator2 {
    private final List<Location> locations;

    public StraightLineEvaluator(List<Location> locations) {
        this.locations = locations;
    }

    @Override
    public long run(int firstIndex, int secondIndex) {
        if(firstIndex == 0 || secondIndex == 0) {
            return 0; //distance to and from depot is 0. This imitates that we don't care where car ends up
        }
        Location firstLocation = locations.get(firstIndex);
        Location secondLocation = locations.get(secondIndex);

        return LocationUtils.getLocationDistance(firstLocation, secondLocation);
    }
}