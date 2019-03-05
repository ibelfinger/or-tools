package hr.ibelfinger.evaluators;

import com.google.ortools.constraintsolver.NodeEvaluator2;
import hr.ibelfinger.Location;

import java.util.List;

public class CapacityEvaluator extends NodeEvaluator2 {
    private final List<Location> locations;

    public CapacityEvaluator(List<Location> locations) {
        this.locations = locations;
    }

    @Override
    public long run(int fromIndex, int toIndex) {
        return locations.get(fromIndex).getLocationType().capacityModifier();
    }
}
