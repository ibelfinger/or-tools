package hr.ibelfinger;

import java.util.Arrays;
import java.util.List;

public class LocationConfig {

    public static List<Location> locations() {
        return Arrays.asList(
                new Location(0, Location.LocationType.DEPOT, 0,0), //we need depot
                new Location(1, Location.LocationType.PICKUP,  25.0942, 55.2029),
                new Location(1, Location.LocationType.DROPOFF, 25.0356, 55.1817),
                new Location(2, Location.LocationType.PICKUP,  25.0773, 55.1404),
                new Location(2, Location.LocationType.DROPOFF, 25.0984, 55.1632),
                new Location(3, Location.LocationType.PICKUP,  25.0417, 55.115),
                new Location(3, Location.LocationType.DROPOFF, 25.1407, 55.1854),
                new Location(4, Location.LocationType.PICKUP,  25.0779, 55.1529),
                new Location(4, Location.LocationType.DROPOFF, 25.0565, 55.2207),
                new Location(5, Location.LocationType.PICKUP,  25.0943, 55.177),
                new Location(5, Location.LocationType.DROPOFF, 25.2251, 55.425),
                new Location(6, Location.LocationType.PICKUP,  25.2004, 55.274),
                new Location(6, Location.LocationType.DROPOFF, 25.0879, 55.1475),
                new Location(7, Location.LocationType.PICKUP,  25.1005, 55.1528),
                new Location(7, Location.LocationType.DROPOFF, 25.078, 55.1473),
                new Location(8, Location.LocationType.PICKUP,  25.0818, 55.1469),
                new Location(8, Location.LocationType.DROPOFF, 25.0336, 55.1733),
                new Location(9, Location.LocationType.PICKUP,  25.2154, 55.4199),
                new Location(9, Location.LocationType.DROPOFF, 25.2116, 55.4137),
                new Location(10, Location.LocationType.PICKUP,  25.2356, 55.3133),
                new Location(10, Location.LocationType.DROPOFF, 25.1105, 55.1423),
                new Location(11, Location.LocationType.PICKUP,  25.1713, 55.3459),
                new Location(11, Location.LocationType.DROPOFF, 25.2627, 55.3215),
                new Location(12, Location.LocationType.PICKUP,  25.0983, 55.1739),
                new Location(12, Location.LocationType.DROPOFF, 25.0674, 55.1418),
                new Location(13, Location.LocationType.PICKUP,  25.1389, 55.208),
                new Location(13, Location.LocationType.DROPOFF, 25.061, 55.1894),
                new Location(14, Location.LocationType.PICKUP,  25.0647, 55.2139),
                new Location(14, Location.LocationType.DROPOFF, 25.2082, 55.2719),
                new Location(15, Location.LocationType.PICKUP,  25.0715, 55.1408),
                new Location(15, Location.LocationType.DROPOFF, 25.2131, 55.2575),
                new Location(16, Location.LocationType.PICKUP,  25.1847, 55.255),
                new Location(16, Location.LocationType.DROPOFF, 25.0479, 55.2037),
                new Location(17, Location.LocationType.PICKUP,  25.1327, 55.2041),
                new Location(17, Location.LocationType.DROPOFF, 25.0704, 55.1334),
                new Location(18, Location.LocationType.PICKUP,  25.1133, 55.1366),
                new Location(18, Location.LocationType.DROPOFF, 25.0974, 55.1639),
                new Location(19, Location.LocationType.PICKUP,  25.0803, 55.1522),
                new Location(19, Location.LocationType.DROPOFF, 25.2141, 55.2832),
                new Location(20, Location.LocationType.PICKUP,  25.0932, 55.1588),
                new Location(20, Location.LocationType.DROPOFF, 25.1188, 55.2028),
                new Location(21, Location.LocationType.PICKUP,  25.1833, 55.2428),
                new Location(21, Location.LocationType.DROPOFF, 25.2119, 55.2747),
                new Location(22, Location.LocationType.PICKUP,  25.0799, 55.1509),
                new Location(22, Location.LocationType.DROPOFF, 24.9966, 55.046),
                new Location(23, Location.LocationType.PICKUP,  25.1992, 55.2711),
                new Location(23, Location.LocationType.DROPOFF, 25.0642, 55.138),
                new Location(24, Location.LocationType.PICKUP,  25.2507, 55.332),
                new Location(24, Location.LocationType.DROPOFF, 25.214, 55.2829),
                new Location(25, Location.LocationType.PICKUP,  25.0454, 55.1539),
                new Location(25, Location.LocationType.DROPOFF, 25.0649, 55.1559),
                new Location(26, Location.LocationType.PICKUP,  25.096, 55.2398),
                new Location(26, Location.LocationType.DROPOFF, 25.0894, 55.1532),
                new Location(27, Location.LocationType.PICKUP,  25.2579, 55.307),
                new Location(27, Location.LocationType.DROPOFF, 25.2125, 55.2378),
                new Location(28, Location.LocationType.PICKUP,  25.0938, 55.1526),
                new Location(28, Location.LocationType.DROPOFF, 25.0985, 55.171),
                new Location(29, Location.LocationType.PICKUP,  25.1181, 55.3913),
                new Location(29, Location.LocationType.DROPOFF, 25.2081, 55.2721),
                new Location(30, Location.LocationType.PICKUP,  25.1861, 55.2637, true),
                new Location(30, Location.LocationType.DROPOFF, 25.1136, 55.3731),
                new Location(31, Location.LocationType.PICKUP,  25.2141, 55.278),
                new Location(31, Location.LocationType.DROPOFF, 25.1861, 55.2743),
                new Location(32, Location.LocationType.PICKUP,  25.1965, 55.2804),
                new Location(32, Location.LocationType.DROPOFF, 25.0659, 55.1382),
                new Location(33, Location.LocationType.PICKUP,  25.1784, 55.2661),
                new Location(33, Location.LocationType.DROPOFF, 25.1956, 55.2775),
                new Location(34, Location.LocationType.PICKUP,  25.2179, 55.2819),
                new Location(34, Location.LocationType.DROPOFF, 25.2281, 55.3277),
                new Location(35, Location.LocationType.PICKUP,  25.0476, 55.1521),
                new Location(35, Location.LocationType.DROPOFF, 25.2265, 55.2842),
                new Location(36, Location.LocationType.PICKUP,  25.1145, 55.3872),
                new Location(36, Location.LocationType.DROPOFF, 25.1193, 55.3957),
                new Location(37, Location.LocationType.PICKUP,  25.1083, 55.378),
                new Location(37, Location.LocationType.DROPOFF, 25.0873, 55.3137),
                new Location(38, Location.LocationType.PICKUP,  25.0396, 55.2444),
                new Location(38, Location.LocationType.DROPOFF, 25.261, 55.3731),
                new Location(39, Location.LocationType.PICKUP,  25.1242, 55.3775),
                new Location(39, Location.LocationType.DROPOFF, 25.1241, 55.4002),
                new Location(40, Location.LocationType.PICKUP,  25.1998, 55.2686),
                new Location(40, Location.LocationType.DROPOFF, 25.2459, 55.3057),
                new Location(41, Location.LocationType.PICKUP,  25.102, 55.1633),
                new Location(41, Location.LocationType.DROPOFF, 25.2098, 55.2705),
                new Location(42, Location.LocationType.PICKUP,  25.0732, 55.1313),
                new Location(42, Location.LocationType.DROPOFF, 25.2484, 55.3524),
                new Location(43, Location.LocationType.PICKUP,  25.187, 55.2781),
                new Location(43, Location.LocationType.DROPOFF, 25.2236, 55.2597),
                new Location(44, Location.LocationType.PICKUP,  25.1876, 55.2676),
                new Location(44, Location.LocationType.DROPOFF, 25.2077, 55.2783),
                new Location(45, Location.LocationType.PICKUP,  25.1808, 55.2653),
                new Location(45, Location.LocationType.DROPOFF, 25.2103, 55.2368),
                new Location(46, Location.LocationType.PICKUP,  25.214, 55.2829),
                new Location(46, Location.LocationType.DROPOFF, 25.2009, 55.2721),
                new Location(47, Location.LocationType.PICKUP,  25.2597, 55.326),
                new Location(47, Location.LocationType.DROPOFF, 25.2278, 55.289),
                new Location(48, Location.LocationType.PICKUP,  25.0953, 55.16),
                new Location(48, Location.LocationType.DROPOFF, 25.0456, 55.2565),
                new Location(49, Location.LocationType.PICKUP,  25.1871, 55.2789),
                new Location(49, Location.LocationType.DROPOFF, 25.1843, 55.2257),
                new Location(50, Location.LocationType.PICKUP,  25.2295, 55.2866),
                new Location(50, Location.LocationType.DROPOFF, 25.2447, 55.3395),
                new Location(51, Location.LocationType.PICKUP,  25.1505, 55.2076),
                new Location(51, Location.LocationType.DROPOFF, 25.1094, 55.1457),
                new Location(52, Location.LocationType.PICKUP,  25.0681, 55.1326),
                new Location(52, Location.LocationType.DROPOFF, 25.2346, 55.3528),
                new Location(53, Location.LocationType.PICKUP,  25.1928, 55.2888),
                new Location(53, Location.LocationType.DROPOFF, 25.2313, 55.3076),
                new Location(54, Location.LocationType.PICKUP,  25.101, 55.1692),
                new Location(54, Location.LocationType.DROPOFF, 25.0899, 55.1522),
                new Location(55, Location.LocationType.PICKUP,  25.1813, 55.2627),
                new Location(55, Location.LocationType.DROPOFF, 25.1894, 55.235),
                new Location(56, Location.LocationType.PICKUP,  25.2213, 55.2812),
                new Location(56, Location.LocationType.DROPOFF, 25.2496, 55.286),
                new Location(57, Location.LocationType.PICKUP,  25.2309, 55.42, true),
                new Location(57, Location.LocationType.DROPOFF, 25.2905, 55.3998),
                new Location(58, Location.LocationType.PICKUP,  25.1883, 55.2976),
                new Location(58, Location.LocationType.DROPOFF, 25.1881, 55.3744),
                new Location(59, Location.LocationType.PICKUP,  25.0962, 55.1762),
                new Location(59, Location.LocationType.DROPOFF, 25.1124, 55.1888),
                new Location(60, Location.LocationType.PICKUP,  25.0784, 55.1487),
                new Location(60, Location.LocationType.DROPOFF, 25.106, 55.1482),
                new Location(61, Location.LocationType.PICKUP,  25.0949, 55.1546, true),
                new Location(61, Location.LocationType.DROPOFF, 25.0758, 55.1398),
                new Location(62, Location.LocationType.PICKUP,  25.272, 55.3161),
                new Location(62, Location.LocationType.DROPOFF, 25.1088, 55.1443),
                new Location(63, Location.LocationType.PICKUP,  25.2357, 55.3267),
                new Location(63, Location.LocationType.DROPOFF, 25.0565, 55.1987),
                new Location(64, Location.LocationType.PICKUP,  25.2111, 55.282),
                new Location(64, Location.LocationType.DROPOFF, 25.1929, 55.2769),
                new Location(65, Location.LocationType.PICKUP,  25.2125, 55.4286),
                new Location(65, Location.LocationType.DROPOFF, 25.2388, 55.3483),
                new Location(66, Location.LocationType.PICKUP,  25.1542, 55.2012),
                new Location(66, Location.LocationType.DROPOFF, 25.1401, 55.2172),
                new Location(67, Location.LocationType.PICKUP,  25.0938, 55.1526),
                new Location(67, Location.LocationType.DROPOFF, 25.1002, 55.1764),
                new Location(68, Location.LocationType.PICKUP,  25.1096, 55.203),
                new Location(68, Location.LocationType.DROPOFF, 25.1181, 55.2006),
                new Location(69, Location.LocationType.PICKUP,  25.1558, 55.2484),
                new Location(69, Location.LocationType.DROPOFF, 25.1019, 55.2105),
                new Location(70, Location.LocationType.PICKUP,  25.1968, 55.2639),
                new Location(70, Location.LocationType.DROPOFF, 25.2227, 55.2891),
                new Location(71, Location.LocationType.PICKUP,  25.1084, 55.3764),
                new Location(71, Location.LocationType.DROPOFF, 25.1182, 55.2155),
                new Location(72, Location.LocationType.PICKUP,  25.1966, 55.2804),
                new Location(72, Location.LocationType.DROPOFF, 24.4392, 54.5966),
                new Location(73, Location.LocationType.PICKUP,  25.2211, 55.285),
                new Location(73, Location.LocationType.DROPOFF, 25.0739, 55.1392),
                new Location(74, Location.LocationType.PICKUP,  25.1102, 55.3814),
                new Location(74, Location.LocationType.DROPOFF, 25.1937, 55.2758),
                new Location(75, Location.LocationType.PICKUP,  25.2395, 55.3671),
                new Location(75, Location.LocationType.DROPOFF, 25.1901, 55.3957),
                new Location(76, Location.LocationType.PICKUP,  25.1991, 55.2751),
                new Location(76, Location.LocationType.DROPOFF, 25.1985, 55.265),
                new Location(77, Location.LocationType.PICKUP,  25.23, 55.3508),
                new Location(77, Location.LocationType.DROPOFF, 25.2661, 55.3246),
                new Location(78, Location.LocationType.PICKUP,  25.0881, 55.1472),
                new Location(78, Location.LocationType.DROPOFF, 25.1977, 55.2744),
                new Location(79, Location.LocationType.PICKUP,  25.2617, 55.2998),
                new Location(79, Location.LocationType.DROPOFF, 24.941, 55.056),
                new Location(80, Location.LocationType.PICKUP,  25.1096, 55.2041),
                new Location(80, Location.LocationType.DROPOFF, 25.0764, 55.1453),
                new Location(81, Location.LocationType.PICKUP,  25.1039, 55.2008),
                new Location(81, Location.LocationType.DROPOFF, 25.1978, 55.2396),
                new Location(82, Location.LocationType.PICKUP,  25.0958, 55.1537),
                new Location(82, Location.LocationType.DROPOFF, 25.0878, 55.1463),
//
                new Location(83, Location.LocationType.PICKUP,  25.0959, 55.1539),
                new Location(83, Location.LocationType.DROPOFF, 25.0957, 55.1538), //This guy
//
                new Location(84, Location.LocationType.PICKUP,  25.2617, 55.2998),
                new Location(84, Location.LocationType.DROPOFF, 24.941, 55.056),
                new Location(85, Location.LocationType.PICKUP,  25.0692, 55.1426),
                new Location(85, Location.LocationType.DROPOFF, 24.4411, 54.5753),
                new Location(86, Location.LocationType.PICKUP,  25.1373, 55.2425),
                new Location(86, Location.LocationType.DROPOFF, 25.0963, 55.1566),
                new Location(87, Location.LocationType.PICKUP,  25.2478, 55.3034),
                new Location(87, Location.LocationType.DROPOFF, 25.1602, 55.3256),
                new Location(88, Location.LocationType.PICKUP,  25.1443, 55.2954),
                new Location(88, Location.LocationType.DROPOFF, 25.2591, 55.2913),
                new Location(89, Location.LocationType.PICKUP,  25.1918, 55.2717),
                new Location(89, Location.LocationType.DROPOFF, 25.1146, 55.1993),

                new Location(90, Location.LocationType.PICKUP,  25.2452, 55.2892),
                new Location(90, Location.LocationType.DROPOFF, 25.2329, 55.2909),
                new Location(91, Location.LocationType.PICKUP,  25.076, 55.1404),
                new Location(91, Location.LocationType.DROPOFF, 25.1107, 55.1419),
                new Location(92, Location.LocationType.PICKUP,  25.1287, 55.2227),
                new Location(92, Location.LocationType.DROPOFF, 25.2138, 55.282),
                new Location(93, Location.LocationType.PICKUP,  25.0336, 55.1733),
                new Location(93, Location.LocationType.DROPOFF, 25.2484, 55.3524),

                new Location(94, Location.LocationType.PICKUP,  25.2059, 55.2751),
                new Location(94, Location.LocationType.DROPOFF, 25.1121, 55.2046),
                new Location(95, Location.LocationType.PICKUP,  25.2452, 55.3132),
                new Location(95, Location.LocationType.DROPOFF, 25.1882, 55.2583),
                new Location(96, Location.LocationType.PICKUP,  25.0972, 55.1748),
                new Location(96, Location.LocationType.DROPOFF, 25.0885, 55.1469),
                new Location(97, Location.LocationType.PICKUP,  25.2088, 55.2726),
                new Location(97, Location.LocationType.DROPOFF, 25.1866, 55.2604),
                new Location(98, Location.LocationType.PICKUP,  25.2581, 55.3029),
                new Location(98, Location.LocationType.DROPOFF, 25.0676, 55.1442),
                new Location(99, Location.LocationType.PICKUP,  25.2545, 55.2899, true),
                new Location(99, Location.LocationType.DROPOFF, 25.1834, 55.2609),
                new Location(100, Location.LocationType.PICKUP,  25.098, 55.1633),
                new Location(100, Location.LocationType.DROPOFF, 25.0997, 55.173)
        );

    }
}
