package hr.ibelfinger;

import java.util.List;

public class LocationUtils {

    public static long getLocationDistance(Location firstLocation, Location secondLocation) {
        return (long) ((Math.abs(firstLocation.getLatitude() - secondLocation.getLatitude())
                + Math.abs(firstLocation.getLongitude() - secondLocation.getLongitude())) * 10_000);
    }

    public static Integer findLocationIndex(List<Location> locations, int bookingId, Location.LocationType locationType) {
        for(int i = 0; i< locations.size(); i++) {
            final Location location = locations.get(i);
            if(location.getLocationType() == locationType && location.getBookingId() == bookingId) {
                return i;
            }
        }
        return null;
    }
}
