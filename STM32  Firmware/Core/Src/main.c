/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "arm_const_structs.h"
#include "string.h"
#include "arm_math.h"

#define EMG_LENGTH 2048
#define NUM_CHANNELS 4
#define Full_Data EMG_LENGTH*NUM_CHANNELS
uint16_t adc_buffer [NUM_CHANNELS * EMG_LENGTH];



char message[50];
void serialPrint(UART_HandleTypeDef *uart, uint8_t *message)
{
  HAL_UART_Transmit(uart, (uint8_t *)message, strlen(message), 500);
}

///////////////multisine
#define SIGNAL_LENGTH 2048
#define SIZE_F 20
uint32_t Excitation[SIGNAL_LENGTH] = {2656, 2310, 1727, 1169, 903, 1068, 1608, 2296, 2848, 3056, 2884, 2461, 1999, 1677, 1546, 1527, 1482, 1317, 1058, 852, 880, 1248, 1900, 2620, 3126, 3201, 2812, 2133, 1470, 1120, 1233, 1749, 2430, 2984, 3197, 3024, 2590, 2113, 1790, 1710, 1832, 2030, 2172, 2185, 2082, 1928, 1793, 1704, 1639, 1550, 1405, 1216, 1029, 896, 844, 856, 887, 903, 906, 940, 1067, 1318, 1667, 2030, 2310, 2447, 2458, 2427, 2460, 2611, 2839, 3019, 2998, 2685, 2121, 1478, 998, 883, 1198, 1832, 2545, 3067, 3221, 2996, 2541, 2093, 1863, 1946, 2292, 2742, 3114, 3284, 3230, 3024, 2778, 2584, 2468, 2393, 2288, 2095, 1805, 1464, 1161, 988, 1013, 1253, 1670, 2180, 2675, 3042, 3197, 3104, 2795, 2360, 1926, 1615, 1503, 1594, 1819, 2072, 2247, 2290, 2207, 2054, 1905, 1817, 1809, 1864, 1952, 2049, 2141, 2215, 2245, 2189, 2007, 1694, 1308, 967, 818, 961, 1399, 2007, 2578, 2905, 2876, 2535, 2059, 1682, 1573, 1756, 2093, 2363, 2373, 2071, 1574, 1117, 935, 1140, 1662, 2285, 2749, 2878, 2655, 2213, 1756, 1440, 1308, 1282, 1237, 1100, 910, 808, 951, 1408, 2089, 2768, 3172, 3123, 2632, 1906, 1263, 982, 1180, 1757, 2454, 2975, 3125, 2885, 2405, 1909, 1586, 1506, 1610, 1764, 1841, 1787, 1632, 1455, 1328, 1271, 1253, 1218, 1132, 1006, 889, 833, 858, 939, 1029, 1092, 1138, 1222, 1405, 1713, 2102, 2468, 2699, 2736, 2614, 2450, 2378, 2474, 2701, 2917, 2943, 2667, 2118, 1471, 982, 868, 1199, 1859, 2592, 3117, 3252, 2995, 2514, 2062, 1857, 1981, 2361, 2811, 3133, 3205, 3029, 2708, 2384, 2161, 2057, 2015, 1941, 1766, 1487, 1170, 927, 859, 1021, 1397, 1909, 2440, 2868, 3100, 3089, 2848, 2444, 1979, 1572, 1316, 1258, 1380, 1608, 1842, 1994, 2022, 1938, 1798, 1674, 1615, 1640, 1735, 1867, 2009, 2137, 2229, 2256, 2177, 1965, 1628, 1238, 918, 807, 994, 1466, 2092, 2661, 2971, 2918, 2548, 2037, 1611, 1441, 1554, 1832, 2073, 2103, 1870, 1475, 1123, 1014, 1234, 1710, 2243, 2605, 2650, 2381, 1933, 1493, 1202, 1086, 1072, 1050, 963, 858, 863, 1108, 1626, 2302, 2905, 3188, 3013, 2433, 1684, 1083, 884, 1166, 1791, 2481, 2947, 3016, 2700, 2169, 1654, 1332, 1255, 1346, 1469, 1504, 1413, 1244, 1082, 999, 1005, 1056, 1092, 1076, 1022, 981, 1001, 1095, 1230, 1354, 1436, 1495, 1594, 1799, 2127, 2518, 2850, 3002, 2921, 2663, 2375, 2221, 2290, 2542, 2812, 2896, 2659, 2123, 1471, 973, 859, 1204, 1883, 2628, 3147, 3258, 2971, 2470, 2021, 1841, 1997, 2390, 2813, 3059, 3019, 2723, 2309, 1943, 1731, 1680, 1704, 1685, 1543, 1286, 1002, 819, 848, 1125, 1603, 2164, 2667, 2990, 3064, 2888, 2516, 2044, 1581, 1226, 1043, 1050, 1210, 1450, 1679, 1826, 1859, 1796, 1689, 1602, 1582, 1646, 1777, 1945, 2114, 2255, 2340, 2338, 2216, 1961, 1596, 1202, 903, 826, 1048, 1541, 2168, 2721, 3007, 2927, 2527, 1981, 1510, 1282, 1336, 1572, 1807, 1879, 1734, 1449, 1196, 1138, 1345, 1747, 2170, 2422, 2390, 2091, 1657, 1257, 1009, 930, 949, 969, 948, 934, 1039, 1362, 1906, 2539, 3037, 3183, 2884, 2234, 1485, 947, 840, 1198, 1852, 2514, 2904, 2881, 2488, 1914, 1390, 1082, 1020, 1116, 1229, 1251, 1158, 1010, 896, 881, 965, 1095, 1202, 1253, 1266, 1290, 1370, 1510, 1670, 1795, 1861, 1897, 1978, 2171, 2488, 2852, 3127, 3186, 2986, 2605, 2219, 2014, 2089, 2390, 2729, 2875, 2676, 2149, 1488, 979, 863, 1218, 1910, 2659, 3164, 3249, 2934, 2420, 1980, 1824, 2002, 2391, 2765, 2919, 2764, 2365, 1890, 1522, 1367, 1404, 1517, 1564, 1459, 1222, 964, 832, 938, 1302, 1842, 2409, 2846, 3038, 2951, 2624, 2153, 1653, 1229, 955, 863, 945, 1151, 1410, 1646, 1803, 1856, 1825, 1757, 1709, 1724, 1815, 1968, 2149, 2318, 2442, 2489, 2432, 2252, 1947, 1557, 1166, 892, 851, 1102, 1606, 2224, 2755, 3014, 2909, 2486, 1912, 1403, 1127, 1134, 1344, 1593, 1726, 1679, 1503, 1332, 1300, 1465, 1770, 2073, 2219, 2127, 1823, 1430, 1093, 910, 886, 953, 1028, 1078, 1149, 1334, 1704, 2234, 2786, 3152, 3153, 2741, 2044, 1322, 865, 853, 1279, 1944, 2557, 2859, 2742, 2280, 1679, 1168, 893, 868, 990, 1118, 1154, 1087, 985, 938, 1001, 1162, 1360, 1525, 1627, 1686, 1753, 1867, 2022, 2171, 2261, 2275, 2258, 2292, 2452, 2738, 3061, 3273, 3243, 2938, 2462, 2013, 1795, 1906, 2276, 2691, 2896, 2726, 2198, 1522, 999, 880, 1241, 1941, 2687, 3175, 3233, 2895, 2375, 1949, 1818, 2008, 2376, 2686, 2742, 2482, 2004, 1507, 1182, 1120, 1271, 1484, 1594, 1516, 1285, 1037, 937, 1098, 1518, 2085, 2625, 2970, 3025, 2789, 2343, 1816, 1333, 984, 817, 830, 990, 1244, 1525, 1773, 1944, 2022, 2022, 1987, 1965, 1996, 2093, 2241, 2405, 2544, 2620, 2603, 2475, 2229, 1881, 1478, 1105, 870, 870, 1148, 1658, 2262, 2769, 3004, 2881, 2443, 1851, 1312, 999, 972, 1171, 1450, 1656, 1710, 1632, 1521, 1488, 1586, 1780, 1962, 2019, 1893, 1617, 1294, 1044, 942, 984, 1105, 1237, 1352, 1492, 1728, 2104, 2580, 3016, 3230, 3088, 2582, 1866, 1199, 841, 927, 1410, 2069, 2618, 2827, 2623, 2110, 1506, 1031, 814, 847, 1014, 1177, 1246, 1222, 1177, 1197, 1329, 1550, 1792, 1989, 2114, 2194, 2276, 2392, 2530, 2636, 2659, 2595, 2503, 2477, 2594, 2846, 3131, 3288, 3187, 2808, 2271, 1798, 1601, 1771, 2220, 2709, 2961, 2804, 2264, 1566, 1027, 905, 1272, 1977, 2716, 3185, 3218, 2865, 2347, 1940, 1830, 2024, 2359, 2596, 2556, 2208, 1684, 1204, 960, 1019, 1296, 1603, 1761, 1688, 1440, 1182, 1095, 1288, 1739, 2308, 2800, 3046, 2972, 2614, 2091, 1555, 1131, 890, 845, 966, 1201, 1494, 1790, 2044, 2224, 2317, 2336, 2315, 2296, 2318, 2392, 2506, 2626, 2706, 2711, 2616, 2414, 2112, 1740, 1349, 1017, 838, 887, 1194, 1706, 2293, 2776, 2992, 2860, 2419, 1817, 1258, 915, 865, 1065, 1382, 1666, 1815, 1820, 1747, 1688, 1701, 1778, 1853, 1847, 1722, 1506, 1281, 1135, 1121, 1228, 1400, 1577, 1740, 1921, 2172, 2515, 2897, 3190, 3246, 2972, 2400, 1700, 1117, 874, 1060, 1589, 2228, 2704, 2821, 2545, 2004, 1423, 1010, 871, 976, 1200, 1406, 1514, 1533, 1539, 1612, 1789, 2039, 2294, 2491, 2611, 2683, 2754, 2848, 2941, 2979, 2916, 2760, 2586, 2503, 2583, 2813, 3076, 3201, 3057, 2636, 2076, 1614, 1463, 1706, 2232, 2779, 3058, 2896, 2332, 1607, 1053, 932, 1308, 2018, 2749, 3199, 3212, 2849, 2340, 1956, 1865, 2053, 2345, 2507, 2378, 1964, 1426, 1000, 868, 1063, 1462, 1847, 2025, 1928, 1639, 1352, 1262, 1472, 1939, 2497, 2934, 3086, 2905, 2461, 1907, 1405, 1075, 961, 1041, 1260, 1551, 1863, 2157, 2401, 2574, 2665, 2681, 2648, 2606, 2591, 2617, 2674, 2728, 2735, 2661, 2490, 2227, 1894, 1529, 1183, 921, 815, 922, 1255, 1765, 2331, 2791, 2994, 2861, 2424, 1820, 1246, 880, 813, 1021, 1383, 1742, 1977, 2046, 1989, 1885, 1805, 1773, 1763, 1728, 1640, 1514, 1403, 1367, 1437, 1596, 1800, 2002, 2186, 2376, 2604, 2876, 3134, 3271, 3174, 2793, 2193, 1547, 1079, 965, 1249, 1812, 2419, 2817, 2849, 2516, 1972, 1438, 1106, 1056, 1238, 1519, 1764, 1902, 1951, 1987, 2086, 2275, 2520, 2755, 2924, 3014, 3059, 3101, 3154, 3189, 3147, 2991, 2745, 2500, 2373, 2438, 2668, 2933, 3053, 2897, 2468, 1918, 1490, 1400, 1714, 2301, 2884, 3164, 2978, 2381, 1630, 1068, 955, 1345, 2062, 2786, 3218, 3215, 2849, 2354, 1993, 1916, 2090, 2332, 2417, 2213, 1757, 1238, 896, 893, 1227, 1732, 2165, 2331, 2180, 1830, 1500, 1400, 1622, 2101, 2645, 3032, 3107, 2846, 2355, 1809, 1377, 1161, 1176, 1368, 1658, 1977, 2280, 2545, 2757, 2903, 2970, 2959, 2891, 2801, 2728, 2691, 2680, 2662, 2596, 2452, 2223, 1928, 1601, 1282, 1017, 852, 833, 998, 1352, 1850, 2388, 2823, 3016, 2888, 2460, 1858, 1272, 886, 807, 1027, 1434, 1863, 2172, 2288, 2227, 2067, 1895, 1771, 1705, 1675, 1656, 1642, 1652, 1717, 1850, 2038, 2247, 2445, 2620, 2786, 2958, 3129, 3247, 3228, 2999, 2549, 1965, 1414, 1087, 1113, 1489, 2073, 2633, 2949, 2903, 2533, 2004, 1535, 1293, 1333, 1584, 1908, 2174, 2323, 2378, 2417, 2509, 2678, 2889, 3078, 3199, 3248, 3257, 3264, 3274, 3246, 3126, 2885, 2565, 2271, 2126, 2202, 2459, 2752, 2894, 2754, 2341, 1821, 1441, 1410, 1780, 2404, 2992, 3250, 3024, 2390, 1620, 1063, 970, 1383, 2109, 2827, 3241, 3224, 2858, 2379, 2038, 1968, 2119, 2305, 2316, 2052, 1580, 1108, 872, 1006, 1468, 2054, 2502, 2621, 2392, 1966, 1591, 1484, 1724, 2219, 2756, 3104, 3121, 2811, 2305, 1797, 1453, 1354, 1482, 1756, 2081, 2390, 2651, 2860, 3019, 3117, 3141, 3084, 2964, 2815, 2677, 2574, 2499, 2421, 2299, 2109, 1855, 1566, 1285, 1053, 900, 849, 921, 1136, 1495, 1966, 2466, 2872, 3056, 2935, 2520, 1922, 1325, 920, 830, 1063, 1513, 2007, 2377, 2524, 2447, 2225, 1970, 1776, 1686, 1695, 1768, 1872, 1995, 2136, 2301, 2484, 2666, 2831, 2970, 3084, 3177, 3232, 3208, 3050, 2723, 2250, 1729, 1311, 1147, 1313, 1769, 2355, 2855, 3084, 2967, 2570, 2072, 1674, 1521, 1639, 1940, 2285, 2549, 2682, 2717, 2731, 2790, 2915, 3070, 3202, 3271, 3280, 3260, 3239, 3211, 3130, 2944, 2637, 2266, 1950, 1818, 1932, 2241, 2584, 2766, 2658, 2278, 1795, 1461, 1474, 1876, 2505, 3070, 3284, 3011, 2345, 1571, 1036, 978, 1420, 2157, 2866, 3261, 3228, 2861, 2396, 2071, 1998, 2118, 2245, 2190, 1883, 1420, 1019, 903, 1172, 1743, 2378, 2804, 2848, 2524, 2020, 1608, 1506, 1776, 2299, 2838, 3160, 3136, 2797, 2299, 1845, 1591, 1594, 1806, 2122, 2440, 2700, 2888, 3020, 3108, 3147, 3118, 3009, 2833, 2626, 2432, 2278, 2160, 2045, 1896, 1694, 1453, 1211, 1016, 902, 880, 948, 1101, 1345, 1686, 2109, 2556, 2927, 3100, 2989, 2588, 1993, 1385, 961, 862, 1109, 1602, 2153, 2573, 2738, 2636, 2354, 2031, 1792, 1707, 1777, 1953, 2170, 2380, 2563, 2721, 2861, 2986, 3093, 3175, 3223, 3226, 3166, 3014, 2747, 2366, 1920, 1508, 1254, 1262, 1561, 2073, 2637, 3059, 3193, 3007, 2594, 2131, 1805, 1729, 1904, 2232, 2570, 2806, 2901, 2895, 2864, 2874, 2942, 3039, 3117, 3146, 3130, 3096, 3062, 3010, 2892, 2659, 2308, 1912, 1602, 1508, 1682, 2059, 2465, 2696, 2627, 2280, 1830, 1527, 1563, 1966, 2570, 3086, 3243, 2925, 2242, 1486, 993, 984, 1460, 2206, 2898, 3267, 3212, 2841, 2384, 2066, 1983, 2065, 2134, 2024, 1694, 1263, 952, 963, 1358, 2012, 2662, 3034, 2982, 2555, 1981, 1549, 1472, 1788, 2349, 2899, 3202, 3148, 2792, 2310, 1910, 1734, 1812, 2070, 2385, 2656, 2834, 2929, 2973, 2987, 2967, 2888, 2733, 2513, 2265, 2039, 1862, 1731, 1613, 1472, 1293, 1099, 938, 857, 878, 990, 1163, 1372, 1615, 1906, 2257, 2638, 2968, 3132, 3033, 2647, 2056, 1437, 995, 888, 1151, 1683, 2285, 2744, 2918, 2787, 2451, 2075, 1814, 1758, 1903, 2179, 2487, 2749, 2932, 3041, 3105, 3150, 3185, 3203, 3183, 3102, 2941, 2690, 2354, 1967, 1595, 1328, 1257, 1434, 1842, 2381, 2892, 3213, 3244, 2990, 2563, 2137, 1874, 1860, 2070, 2398, 2707, 2895, 2937, 2878, 2797, 2755, 2772, 2821, 2864, 2874, 2857, 2831, 2802, 2743, 2603, 2339, 1964, 1567, 1285, 1247, 1495, 1946, 2414, 2692, 2655, 2332, 1902, 1610, 1641, 2016, 2568, 3020, 3118, 2767, 2091, 1377, 946, 996, 1505, 2249, 2913, 3243, 3159, 2776, 2321, 2003, 1904, 1946, 1963, 1813, 1483, 1107, 902, 1040, 1544, 2251, 2883, 3175, 3012, 2487, 1860, 1431, 1400, 1777, 2382, 2943, 3228, 3145, 2772, 2303, 1945, 1825, 1945, 2210, 2487, 2680, 2760, 2755, 2714, 2666, 2602, 2491, 2310, 2069, 1810, 1582, 1417, 1309, 1221, 1117, 991, 870, 809, 852, 1007, 1237, 1488, 1715, 1917, 2128, 2384, 2688, 2975, 3134, 3052, 2684, 2099, 1470, 1013, 900, 1180, 1749, 2395, 2884, 3058, 2896, 2513, 2101, 1838, 1825, 2048, 2408, 2775, 3047, 3185, 3210, 3174, 3129, 3093, 3055, 2980, 2834, 2598, 2282, 1921, 1574, 1315, 1217, 1329, 1655, 2138, 2663, 3087, 3286, 3205, 2884, 2447, 2056, 1848, 1878, 2104, 2413, 2676, 2808, 2795, 2686, 2561, 2481, 2464, 2489, 2523, 2542, 2547, 2548, 2538, 2483, 2330, 2048, 1665, 1281, 1042, 1068, 1389, 1908, 2429, 2741, 2723, 2410, 1981, 1678, 1679, 2001, 2485, 2867, 2915, 2553, 1910, 1263, 911, 1021, 1554, 2282, 2900, 3177, 3053, 2651, 2193, 1872, 1754, 1760, 1737, 1568, 1262, 962, 872, 1131, 1722, 2452, 3034, 3225, 2948, 2337, 1679, 1278, 1313, 1759, 2407, 2972, 3229, 3108, 2711, 2244, 1912, 1822, 1954, 2194, 2408, 2509, 2489, 2396, 2292, 2206, 2125, 2009, 1831, 1601, 1363, 1169, 1048, 991, 958, 913, 852, 813, 854, 1014, 1282, 1599, 1887, 2090, 2213, 2312, 2459, 2683, 2934, 3096, 3038, 2695, 2119, 1481, 1011, 896, 1194, 1798, 2481, 2991, 3158, 2965, 2543, 2107, 1857, 1892, 2184, 2604, 2993, 3232, 3288, 3202, 3058, 2927, 2837, 2764, 2659, 2475, 2196, 1852, 1505, 1234, 1113, 1190, 1469, 1911, 2424, 2892, 3195, 3253, 3056, 2672, 2230, 1876, 1717, 1779, 2007, 2287, 2503, 2582, 2523, 2381, 2232, 2135, 2110, 2138, 2189, 2241, 2287, 2326, 2340, 2289, 2125, 1829, 1444, 1082, 888, 977, 1364, 1937, 2493, 2821, 2806, 2485, 2039, 1705, 1657, 1912, 2321, 2639};
int f_freqbins_map[SIZE_F] = {1, 14, 28, 41, 55, 68, 82, 95, 108, 122, 135, 149, 162, 175, 189, 202, 216, 229, 243, 256};

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;
DMA_HandleTypeDef hdma_adc3;

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac_ch1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint32_t Emg1Buffer1[EMG_LENGTH] = {0};
uint32_t Emg2Buffer1[EMG_LENGTH] = {0};
uint32_t Emg3Buffer1[EMG_LENGTH] = {0};
uint32_t Emg4Buffer1[EMG_LENGTH] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t DmaCpltFlag1 = 0;
char msg[100];
uint16_t AdcBuffer1[SIGNAL_LENGTH] = { 0 };
uint16_t AdcBuffer2[SIGNAL_LENGTH] = { 0 };
float32_t accPhase[SIZE_F];
float32_t accMagnitude[SIZE_F];
float32_t accFactor = 0.999;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC3_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_DAC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /***************************************************************SETTING POTENTIOMETER***/
  //EMG1
  // MODE=0 → common; DACSEL ignored
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
  // Move UP 10 steps (both wipers, since MODE=0)
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // U/D = UP (toward A)
  for (int i = 0; i < 10; i++) {                           // 10 steps (10 fall edge triggers for the clck)
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); // falling edge = step
  }


  //EMG2
  // MODE=0 → common; DACSEL ignored
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
  // Move UP 10 steps (both wipers, since MODE=0)
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET); // U/D = UP (toward A)
  for (int i = 0; i < 10; i++) {                           // 10 steps (10 fall edge triggers for the clck)
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); // falling edge = step
  }

  //EMG3
  // MODE=0 → common; DACSEL ignored
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
  // Move UP 10 steps (both wipers, since MODE=0)
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET); // U/D = UP (toward A)
  for (int i = 0; i < 10; i++) {                           // 10 steps (10 fall edge triggers for the clck)
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET); // falling edge = step
  }

  //EMG4
  // MODE=0 → common; DACSEL ignored
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
  // Move UP 10 steps (both wipers, since MODE=0)
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET); // U/D = UP (toward A)
  for (int i = 0; i < 10; i++) {                           // 10 steps (10 fall edge triggers for the clck)
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET); // falling edge = step
  }



  //BIS

  #define FFT
//#define GOERTZEL
//	#define SIGNAL_OUT

//#define EMG
#define SIGNAL_OUT_SERIALPLOT_STYLE


  	  	// Start DMA / DAC 3 bio
		HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, Excitation, SIGNAL_LENGTH,DAC_ALIGN_12B_R);
		HAL_TIM_Base_Start(&htim4);


		// Calibrate ADCs bio
		HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
		HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

		// Start DMA & ADC 1/2 bio
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*) AdcBuffer1, SIGNAL_LENGTH);
		HAL_ADC_Start_DMA(&hadc2, (uint32_t*) AdcBuffer2, SIGNAL_LENGTH);
		HAL_TIM_Base_Start(&htim2);

		  //EMG
		HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);
	    HAL_ADC_Start_DMA(&hadc3, (uint32_t *)adc_buffer, Full_Data);
	    HAL_TIM_Base_Start(&htim3);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	    if ((DmaCpltFlag1 == 1))
	    {
	        DmaCpltFlag1 = 0;

	  	      HAL_TIM_Base_Stop(&htim2);

	  	    HAL_TIM_Base_Stop(&htim3);

		      for (int i = 0; i < EMG_LENGTH; ++i) {
		    	  Emg1Buffer1[i] = adc_buffer[i * 4];
		    	  Emg2Buffer1[i] = adc_buffer[i * 4 + 1];
		    	  Emg3Buffer1[i] = adc_buffer[i * 4 + 2];
		    	  Emg4Buffer1[i] = adc_buffer[i * 4 + 3];
		      }

	    	////////////////////////////////////////////////////// Out signal /////////////////////////////////
	    	#ifdef SIGNAL_OUT
	    	      char message[50];
	    	      int cnt = 0;

	    	      serialPrint(&huart2, "Signal writing...\r\n");
	    	      for (cnt = 0; cnt < SIGNAL_LENGTH; cnt++)
	    	      {

	    	#ifdef SIGNAL_OUT_SERIALPLOT_STYLE
	    	        sprintf(message, "%d,%d,%d\r\n",cnt, AdcBuffer1[cnt], AdcBuffer2[cnt]); //, Emg1Buffer1[cnt], Emg2Buffer1[cnt], Emg3Buffer1[cnt], Emg4Buffer1[cnt] /, %u, %u, %u, %u
	    	        HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen(message), 500);

	    	#else
	    	        sprintf(message, "ADC1(%4d)=%4d;\r\n", cnt + 1, AdcBuffer1[cnt]);
	    	        HAL_UART_Transmit(&huart2, (uint8_t *)message, 18, 500);

	    	        sprintf(message, "ADC2(%4d)=%4d;\r\n", cnt + 1, AdcBuffer2[cnt]);
	    	        HAL_UART_Transmit(&huart2, (uint8_t *)message, 18, 500);

	    	#endif

	    	        //

	    	        //	  	  		 	HAL_Delay(10);
	    	      }
	    	      serialPrint(&huart2, "Signal writing done\r\n");
	    	#endif








#ifdef GOERTZEL

      uint8_t message[50];
		#include "goertzel.h"
      float32_t Magnitude[SIZE_F];
      float32_t Phase[SIZE_F];
      goertzel_struct g1[SIZE_F];
      goertzel_struct_singlesideres gres1[SIZE_F];
      goertzel_struct g2[SIZE_F];
      goertzel_struct_singlesideres gres2[SIZE_F];

	  for(int k = 0; k< SIZE_F; k++) {
		  goertzel_init2(f_freqbins_map[k], SIGNAL_LENGTH, &g1[k]);
		  goertzel_init2(f_freqbins_map[k], SIGNAL_LENGTH, &g2[k]);

		  for(int i=0; i<SIGNAL_LENGTH; i++) {
			  goertzel(&g1[k], (double)AdcBuffer1[i]);
			    goertzel(&g2[k], (double)AdcBuffer2[i]);

			  //goertzel(&g1[k], (double)AdcBuffer1[i]/ACQ_LENGTH * powf(arm_sin_f32(pi*(float)i/ACQ_LENGTH),2));
			  //goertzel(&g2[k], (double)AdcBuffer2[i]/FFT_LENGTH * powf(arm_sin_f32(pi*(float)i/ACQ_LENGTH),2));
			//  goertzel(&g1[k], (float)AdcBuffer1[i]/ACQ_LENGTH * powf(arm_sin_f32(pi*(float)i/ACQ_LENGTH),2));
			//  goertzel(&g2[k], (float)AdcBuffer2[i]/ACQ_LENGTH * powf(arm_sin_f32(pi*(float)i/ACQ_LENGTH),2));

		  }

		  goertzel_end(&g1[k], &gres1[k], SIGNAL_LENGTH);
		  goertzel_end(&g2[k], &gres2[k], SIGNAL_LENGTH);

		  Magnitude[k] = 1000.0 *gres1[k].M / gres2[k].M;
//		 Phase[k] =  fmod((float)((gres1[k].Phi-gres2[k].Phi)*(180/PI))+180 - 360, 360);  //+90;
	  Phase[k] =  fmod((float)((gres1[k].Phi-gres2[k].Phi)*(180/PI))+360-360, 360);  //+90;



		if (accPhase[k] == 0) accPhase[k] = Phase[k];
		if (accMagnitude[k] == 0) accMagnitude[k] = Magnitude[k];
		accPhase[k] = accPhase[k] * accFactor + Phase[k] * (1-accFactor);
		accMagnitude[k] = accMagnitude[k] * accFactor + Magnitude[k] * (1-accFactor);
	}

		      for (int cnt = 0; cnt < SIZE_F; cnt++) {

		#ifdef SIGNAL_OUT_SERIALPLOT_STYLE
		    	  //Magnitude, Phase, accumulated magnitude, accumulated phase
		        sprintf(message, "%d, %d.%d,%d.%d,%d.%d,%d.%d\r\n", cnt+1, (int)(Magnitude[cnt]),  (int)((fabs(Magnitude[cnt]) - (int)fabs(Magnitude[cnt])) * 1000),
		        		(int)Phase[cnt], (int)((fabs(Phase[cnt]) - (int)fabs(Phase[cnt])) * 1000),
						(int)(accMagnitude[cnt]),  (int)((fabs(accMagnitude[cnt]) - (int)fabs(accMagnitude[cnt])) * 1000),
						(int)accPhase[cnt], (int)((fabs(accPhase[cnt]) - (int)fabs(accPhase[cnt])) * 1000)

		        );
		        HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen(message), 500);

		#else
		        // Magnitude, Phase from UART
		        sprintf(message, "Phas(%02d)=%d.%d;\r\n", cnt + 1, (int)Phase[cnt], (int)((fabs(Phase[cnt]) - (int)fabs(Phase[cnt])) * 1000));
		        HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen(message), 500);
		        sprintf(message, "Magn(%02d)=%d.%d;\r\n", cnt + 1, (int)Magnitude[cnt], (int)((fabs(Magnitude[cnt]) - (int)fabs(Magnitude[cnt])) * 1000));
		        HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen(message), 500);
		        sprintf(message, "AccPhas(%02d)=%d.%d;\r\n", cnt + 1, (int)accPhase[cnt], (int)((fabs(accPhase[cnt]) - (int)fabs(accPhase[cnt])) * 1000));
			   HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen(message), 500);
			   sprintf(message, "AccMagn(%02d)=%d.%d;\r\n", cnt + 1, (int)accMagnitude[cnt], (int)((fabs(accMagnitude[cnt]) - (int)fabs(accMagnitude[cnt])) * 1000));
			   HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen(message), 500);
		#endif
		      }



#endif



#ifdef EMG
	//      uint32_t start_time = HAL_GetTick(); // Start timestamp in ms

	      for (int i = 0; i < EMG_LENGTH; i++)
	      {
	          sprintf(msg, "EMG1 %d: %u\r\n", i, Emg1Buffer1[i]);
	          HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 10);

	          sprintf(msg, "EMG2 %d: %u\r\n", i, Emg2Buffer1[i]);
	          HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 10);

	          sprintf(msg, "EMG3 %d: %u\r\n", i, Emg3Buffer1[i]);
	          HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 10);

	          sprintf(msg, "EMG4 %d: %u\r\n", i, Emg4Buffer1[i]);
	          HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 10);
	      }
#endif

			////////////////////////////////////////////////////////// FFT ///////////////////////////////////////////////////////
#ifdef FFT

			      uint8_t message[50];

			      float32_t Magnitude[SIZE_F];
			      float32_t Magnitude1[SIZE_F];
			      float32_t Magnitude2[SIZE_F];
			      float32_t Phase[SIZE_F];
			      float32_t Theta1[SIZE_F];
			      float32_t Theta2[SIZE_F];
			      float32_t in1;
			      float32_t in2;
			      float32_t input1;
				  float32_t input2;
				  int idxtest;


			     float32_t fft_inputbuf[SIGNAL_LENGTH * 2];
			     // int16_t fft_inputbuf[SIGNAL_LENGTH * 2];
			      for (int i = 0; i < SIGNAL_LENGTH; i++)
			      {

			        fft_inputbuf[i * 2] = AdcBuffer1[i];
			        fft_inputbuf[i * 2 + 1] = 0;
			      }
			      arm_cfft_f32(&arm_cfft_sR_f32_len2048, fft_inputbuf, 0, 1);
			      /*
			      if(SIGNAL_LENGTH==2048)       arm_cfft_f32(&arm_cfft_sR_f32_len2048, fft_inputbuf, 0, 1);
			      else if(SIGNAL_LENGTH == 4096) arm_cfft_f32(&arm_cfft_sR_f32_len4096, fft_inputbuf, 0, 1);
*/
			  //     if(ACQ_LENGTH==2048)       arm_cfft_q15(&arm_cfft_sR_q15_len2048, fft_inputbuf, 0, 1);
			    //     else if(ACQ_LENGTH == 4096) arm_cfft_q15(&arm_cfft_sR_q15_len4096, fft_inputbuf, 0, 1);


			      for (int idx = 0; idx < SIZE_F; idx++)
			      			      {

			         in1 = fft_inputbuf[f_freqbins_map[idx] * 2];
			         in2 = fft_inputbuf[f_freqbins_map[idx] * 2 + 1];
			        Theta1[idx] = atan2(in2, in1);

			         input1 = fft_inputbuf[f_freqbins_map[idx] * 2];
			         input2 = fft_inputbuf[f_freqbins_map[idx] * 2 + 1];
			        Magnitude1[idx] = (float)sqrt((input1 * input1) + (input2 * input2));

			      			      }

			      // FFT_ADC2_ START
			      for (int i = 0; i < SIGNAL_LENGTH; i++)
			      {

			        fft_inputbuf[i * 2] = AdcBuffer2[i];
			        fft_inputbuf[i * 2 + 1] = 0;
			      }

			      arm_cfft_f32(&arm_cfft_sR_f32_len2048, fft_inputbuf, 0, 1);

			//      if(SIGNAL_LENGTH==2048) arm_cfft_f32(&arm_cfft_sR_f32_len2048, fft_inputbuf, 0, 1);
			   //   else if(SIGNAL_LENGTH == 4096) arm_cfft_f32(&arm_cfft_sR_f32_len4096, fft_inputbuf, 0, 1);

			      //     if(ACQ_LENGTH==2048)       arm_cfft_q15(&arm_cfft_sR_q15_len2048, fft_inputbuf, 0, 1);
			        //     else if(ACQ_LENGTH == 4096) arm_cfft_q15(&arm_cfft_sR_q15_len4096, fft_inputbuf, 0, 1);


			      for (int idx = 0; idx < SIZE_F; idx++)
			      			      {
			        in1 = fft_inputbuf[f_freqbins_map[idx] * 2];
			        in2 = fft_inputbuf[f_freqbins_map[idx] * 2 + 1];
			        Theta2[idx] = atan2(in2, in1);

			        input1 = fft_inputbuf[f_freqbins_map[idx] * 2];
			        input2 = fft_inputbuf[f_freqbins_map[idx] * 2 + 1];

			        Magnitude2[idx] = (float)sqrt((input1 * input1) + (input2 * input2));

			        Phase[idx] = (float)((Theta1[idx] - Theta2[idx]) * (180 / PI)) + 360; //+90;
			        Phase[idx] = fmod(Phase[idx] - 360, 360);
			        Magnitude[idx] = (float)(Magnitude1[idx] / Magnitude2[idx]) * 1000;
			      			      }


			      for (int cnt = 0; cnt < SIZE_F; cnt++) {
			     		#ifdef SIGNAL_OUT_SERIALPLOT_STYLE
			     		    	//Magnitude, Phase, accumulated magnitude, accumulated phase
			     		        sprintf(message, "%d, %d.%d, %d.%d, %d.%d, %d.%d, %u, %u, %u, %u \r\n",
			     		        		cnt+1,
										(int)(Magnitude[cnt]),  (int)((fabs(Magnitude[cnt]) - (int)fabs(Magnitude[cnt])) * 1000),
			     		        		(int)Phase[cnt], (int)((fabs(Phase[cnt]) - (int)fabs(Phase[cnt])) * 1000),
			     						(int)(accMagnitude[cnt]),  (int)((fabs(accMagnitude[cnt]) - (int)fabs(accMagnitude[cnt])) * 1000),
			     						(int)accPhase[cnt], (int)((fabs(accPhase[cnt]) - (int)fabs(accPhase[cnt])) * 1000),
										Emg1Buffer1[cnt],
										Emg2Buffer1[cnt],
										Emg3Buffer1[cnt],
										Emg4Buffer1[cnt]
			     		        );

			     		        HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen(message), 500);

			     		#else
			     		        // Magnitude, Phase from UART
			     		        sprintf(message, "Phas(%02d)=%d.%d;\r\n", cnt + 1, (int)Phase[cnt], (int)((fabs(Phase[cnt]) - (int)fabs(Phase[cnt])) * 1000));
			     		        HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen(message), 500);
			     		        sprintf(message, "Magn(%02d)=%d.%d;\r\n", cnt + 1, (int)Magnitude[cnt], (int)((fabs(Magnitude[cnt]) - (int)fabs(Magnitude[cnt])) * 1000));
			     		        HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen(message), 500);
			     		        sprintf(message, "AccPhas(%02d)=%d.%d;\r\n", cnt + 1, (int)accPhase[cnt], (int)((fabs(accPhase[cnt]) - (int)fabs(accPhase[cnt])) * 1000));
			     			   HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen(message), 500);
			     			   sprintf(message, "AccMagn(%02d)=%d.%d;\r\n", cnt + 1, (int)accMagnitude[cnt], (int)((fabs(accMagnitude[cnt]) - (int)fabs(accMagnitude[cnt])) * 1000));
			     			   HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen(message), 500);
			     		#endif
			     		      }



#endif


			//  	 sprintf(message, "toc;\r\n");
		  	//HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
			      //test		//HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, Excitation, SIGNAL_LENGTH,DAC_ALIGN_12B_R);
			HAL_TIM_Base_Start(&htim2);
	         HAL_TIM_Base_Start(&htim3);
	    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 10;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T2_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T2_TRGO;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.NbrOfConversion = 4;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T3_TRGO;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T4_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10D19CE4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 39;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 99;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 39;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA2_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA6 PA7 PA8 PA9
                           PA10 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC5 PC6 PC7
                           PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  DmaCpltFlag1 = 1;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
