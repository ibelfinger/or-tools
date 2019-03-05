package hr.ibelfinger;

public class Location {
    private final int bookingId;
    private final double latitude;
    private final double longitude;
    private final LocationType locationType;
    private boolean cnf;

    public Location(int bookingId, LocationType locationType, double latitude, double longitude, boolean cnf) {
        this(bookingId, locationType, latitude, longitude);
        this.cnf = cnf;
    }

    public Location(int bookingId, LocationType locationType, double latitude, double longitude) {
        this.bookingId = bookingId;
        this.locationType = locationType;
        this.latitude = latitude;
        this.longitude = longitude;
    }

    public int getBookingId() {
        return bookingId;
    }

    public double getLatitude() {
        return latitude;
    }

    public double getLongitude() {
        return longitude;
    }

    public LocationType getLocationType() {
        return locationType;
    }

    public boolean isCnf() {
        return cnf;
    }

    public enum LocationType {
        DEPOT {
            @Override
            public int capacityModifier() {
                return 0;
            }
        },
        PICKUP {
            @Override
            public int capacityModifier() {
                return 1;
            }
        },
        DROPOFF {
            @Override
            public int capacityModifier() {
                return 0;
            }
        };

        public abstract int capacityModifier();
    }

    @Override
    public String toString() {
        return locationType.name().substring(0,1) + bookingId;
    }
}
