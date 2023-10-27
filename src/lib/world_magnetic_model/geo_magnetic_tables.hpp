/****************************************************************************
 *
 *   Copyright (c) 2020-2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <stdint.h>

static constexpr float SAMPLING_RES = 10;
static constexpr float SAMPLING_MIN_LAT = -90;
static constexpr float SAMPLING_MAX_LAT = 90;
static constexpr float SAMPLING_MIN_LON = -180;
static constexpr float SAMPLING_MAX_LON = 180;

static constexpr int LAT_DIM = 19;
static constexpr int LON_DIM = 37;


// *INDENT-OFF*
// Magnetic declination data in radians * 10^-4
// Model: WMM-2020,
// Version: 0.5.1.11,
// Date: 2023.8192,
static constexpr const int16_t declination_table[19][37] {
	//    LONGITUDE:   -180,  -170,  -160,  -150,  -140,  -130,  -120,  -110,  -100,   -90,   -80,   -70,   -60,   -50,   -40,   -30,   -20,   -10,     0,    10,    20,    30,    40,    50,    60,    70,    80,    90,   100,   110,   120,   130,   140,   150,   160,   170,   180,
	/* LAT: -90 */ {  25955, 24210, 22464, 20719, 18974, 17228, 15483, 13738, 11992, 10247,  8502,  6756,  5011,  3266,  1521,  -225, -1970, -3715, -5461, -7206, -8951,-10696,-12442,-14187,-15932,-17678,-19423,-21168,-22914,-24659,-26405,-28150,-29895, 31191, 29446, 27700, 25955, },
	/* LAT: -80 */ {  22518, 20391, 18454, 16682, 15043, 13506, 12043, 10632,  9258,  7908,  6577,  5259,  3950,  2645,  1337,    17, -1325, -2699, -4111, -5565, -7063, -8602,-10181,-11802,-13469,-15189,-16976,-18851,-20835,-22956,-25232,-27667,-30235, 29960, 27345, 24844, 22518, },
	/* LAT: -70 */ {  14986, 13586, 12456, 11491, 10619,  9785,  8941,  8051,  7098,  6078,  5008,  3915,  2832,  1779,   758,  -251, -1290, -2399, -3605, -4910, -6296, -7731, -9182,-10626,-12054,-13472,-14903,-16392,-18022,-19946,-22485,-26309, 30604, 24097, 19618, 16856, 14986, }, // WARNING! black out zone
	/* LAT: -60 */ {   8453,  8201,  7913,  7633,  7374,  7116,  6804,  6367,  5749,  4926,  3923,  2812,  1695,   673,  -202,  -954, -1681, -2509, -3522, -4731, -6072, -7448, -8773, -9984,-11047,-11944,-12658,-13151,-13323,-12875,-10762, -3403,  5031,  7731,  8481,  8596,  8453, }, // WARNING! black out zone
	/* LAT: -50 */ {   5510,  5544,  5484,  5389,  5311,  5270,  5232,  5101,  4753,  4085,  3070,  1795,   457,  -713, -1570, -2120, -2509, -2952, -3653, -4688, -5950, -7237, -8381, -9279, -9869,-10098, -9892, -9124, -7605, -5236, -2326,   424,  2534,  3960,  4835,  5308,  5510, },
	/* LAT: -40 */ {   3973,  4065,  4069,  4019,  3955,  3918,  3920,  3906,  3729,  3188,  2156,   708,  -853, -2155, -2999, -3432, -3601, -3653, -3835, -4437, -5445, -6535, -7417, -7944, -8036, -7640, -6735, -5344, -3636, -1942,  -488,   731,  1777,  2647,  3308,  3741,  3973, },
	/* LAT: -30 */ {   2999,  3084,  3110,  3091,  3028,  2945,  2882,  2847,  2718,  2233,  1185,  -344, -1949, -3188, -3908, -4236, -4298, -4069, -3631, -3443, -3851, -4617, -5309, -5642, -5497, -4893, -3923, -2719, -1523,  -591,    80,   678,  1305,  1915,  2431,  2797,  2999, },
	/* LAT: -20 */ {   2356,  2401,  2414,  2409,  2362,  2263,  2152,  2073,  1927,  1425,   354, -1145, -2620, -3664, -4174, -4271, -4050, -3472, -2604, -1828, -1590, -1970, -2622, -3078, -3098, -2726, -2084, -1273,  -509,   -32,   223,   522,   967,  1455,  1885,  2199,  2356, },
	/* LAT: -10 */ {   1963,  1956,  1929,  1919,  1886,  1796,  1680,  1586,  1400,   844,  -230, -1614, -2880, -3689, -3931, -3682, -3093, -2305, -1460,  -716,  -270,  -324,  -806, -1308, -1510, -1411, -1094,  -599,  -111,   119,   159,   315,   691,  1140,  1543,  1839,  1963, },
	/* LAT:   0 */ {   1749,  1714,  1653,  1638,  1622,  1547,  1435,  1316,  1060,   436,  -611, -1839, -2883, -3449, -3424, -2906, -2132, -1352,  -706,  -175,   231,   329,    37,  -380,  -628,  -680,  -582,  -323,   -40,    40,   -31,    53,   401,   855,  1286,  1615,  1749, },
	/* LAT:  10 */ {   1610,  1617,  1570,  1581,  1602,  1547,  1418,  1225,   848,   129,  -893, -1966, -2780, -3101, -2877, -2259, -1481,  -774,  -266,   111,   433,   574,   403,    80,  -151,  -258,  -286,  -210,  -112,  -161,  -309,  -285,    22,   488,   979,  1395,  1610, },
	/* LAT:  20 */ {   1418,  1566,  1624,  1712,  1795,  1770,  1609,  1301,   756,   -99, -1139, -2083, -2666, -2760, -2419, -1811, -1099,  -454,     0,   305,   562,   703,   605,   361,   165,    45,   -53,  -126,  -212,  -408,  -651,  -711,  -471,   -15,   534,  1058,  1418, },
	/* LAT:  30 */ {   1106,  1472,  1731,  1954,  2112,  2120,  1928,  1502,   774,  -251, -1361, -2226, -2628, -2555, -2149, -1567,  -911,  -296,   161,   463,   692,   834,   808,   658,   510,   384,   218,    -4,  -294,  -674, -1045, -1203, -1039,  -607,   -26,   585,  1106, },
	/* LAT:  40 */ {    736,  1321,  1816,  2210,  2459,  2503,  2286,  1752,   843,  -377, -1603, -2455, -2764, -2606, -2157, -1563,  -910,  -282,   224,   585,   852,  1046,  1134,  1121,  1045,   893,   615,   191,  -358,  -969, -1489, -1729, -1607, -1187,  -589,    80,   736, },
	/* LAT:  50 */ {    431,  1175,  1858,  2419,  2790,  2899,  2673,  2023,   889,  -593, -1998, -2898, -3185, -2991, -2500, -1854, -1145,  -449,   164,   666,  1081,  1432,  1706,  1873,  1888,  1694,  1236,   511,  -395, -1299, -1972, -2250, -2120, -1680, -1050,  -327,   431, },
	/* LAT:  60 */ {    213,  1061,  1868,  2569,  3081,  3302,  3099,  2302,   812, -1117, -2811, -3782, -4034, -3777, -3208, -2465, -1641,  -804,     1,   750,  1439,  2065,  2603,  2993,  3144,  2945,  2286,  1147,  -281, -1597, -2454, -2753, -2585, -2097, -1415,  -626,   213, },
	/* LAT:  70 */ {    -62,   867,  1763,  2565,  3187,  3494,  3267,  2181,    27, -2609, -4542, -5383, -5411, -4944, -4186, -3260, -2242, -1182,  -111,   945,  1967,  2930,  3794,  4496,  4934,  4950,  4315,  2804,   580, -1531, -2816, -3252, -3094, -2571, -1834,  -978,   -62, }, // WARNING! black out zone
	/* LAT:  80 */ {   -901,    18,   866,  1557,  1951,  1807,   730, -1618, -4589, -6687, -7508, -7448, -6871, -5990, -4925, -3745, -2492, -1197,   121,  1444,  2754,  4035,  5262,  6401,  7399,  8162,  8503,  8037,  6015,  2033, -1670, -3366, -3699, -3353, -2670, -1820,  -901, }, // WARNING! black out zone
	/* LAT:  90 */ { -29351,-27606,-25860,-24115,-22370,-20624,-18879,-17134,-15388,-13643,-11898,-10153, -8408, -6663, -4917, -3172, -1427,   318,  2063,  3808,  5554,  7299,  9044, 10789, 12535, 14280, 16026, 17771, 19516, 21262, 23007, 24753, 26498, 28244, 29989,-31097,-29351, }, // WARNING! black out zone
};
static constexpr float WMM_DECLINATION_MIN_RAD = -3.110; // latitude: 90, longitude: 170
static constexpr float WMM_DECLINATION_MAX_RAD = 3.119; // latitude: -90, longitude: 150


// Magnetic inclination data in radians * 10^-4
// Model: WMM-2020,
// Version: 0.5.1.11,
// Date: 2023.8192,
static constexpr const int16_t inclination_table[19][37] {
	//    LONGITUDE:   -180,  -170,  -160,  -150,  -140,  -130,  -120,  -110,  -100,   -90,   -80,   -70,   -60,   -50,   -40,   -30,   -20,   -10,     0,    10,    20,    30,    40,    50,    60,    70,    80,    90,   100,   110,   120,   130,   140,   150,   160,   170,   180,
	/* LAT: -90 */ { -12564,-12564,-12564,-12564,-12564,-12564,-12564,-12563,-12563,-12563,-12563,-12563,-12563,-12563,-12563,-12563,-12563,-12563,-12563,-12563,-12563,-12563,-12563,-12563,-12563,-12563,-12563,-12564,-12564,-12564,-12564,-12564,-12564,-12564,-12564,-12564,-12564, },
	/* LAT: -80 */ { -13646,-13512,-13351,-13171,-12977,-12776,-12573,-12373,-12182,-12007,-11851,-11717,-11607,-11520,-11457,-11415,-11395,-11397,-11424,-11479,-11562,-11677,-11823,-11997,-12195,-12413,-12642,-12875,-13103,-13315,-13501,-13651,-13754,-13804,-13800,-13745,-13646, },
	/* LAT: -70 */ { -14093,-13774,-13455,-13132,-12801,-12458,-12103,-11747,-11405,-11098,-10847,-10665,-10554,-10501,-10487,-10489,-10493,-10501,-10523,-10578,-10687,-10863,-11111,-11427,-11802,-12221,-12670,-13134,-13600,-14053,-14469,-14812,-14996,-14939,-14707,-14409,-14093, }, // WARNING! black out zone
	/* LAT: -60 */ { -13511,-13157,-12818,-12485,-12141,-11769,-11355,-10901,-10435,-10007, -9680, -9509, -9509, -9648, -9851,-10039,-10158,-10195,-10177,-10160,-10208,-10371,-10665,-11078,-11580,-12138,-12726,-13322,-13911,-14472,-14970,-15258,-15076,-14688,-14280,-13885,-13511, }, // WARNING! black out zone
	/* LAT: -50 */ { -12493,-12149,-11817,-11494,-11171,-10825,-10426, -9955, -9427, -8909, -8525, -8407, -8616, -9081, -9643,-10148,-10498,-10648,-10607,-10448,-10306,-10318,-10548,-10973,-11520,-12115,-12701,-13238,-13682,-13978,-14085,-14011,-13807,-13523,-13193,-12844,-12493, },
	/* LAT: -40 */ { -11239,-10888,-10539,-10192, -9853, -9516, -9156, -8732, -8213, -7651, -7233, -7205, -7679, -8509, -9429,-10252,-10900,-11321,-11443,-11260,-10911,-10645,-10651,-10945,-11413,-11919,-12361,-12678,-12833,-12837,-12748,-12612,-12435,-12203,-11917,-11587,-11239, },
	/* LAT: -30 */ {  -9603, -9219, -8836, -8444, -8052, -7678, -7325, -6937, -6426, -5817, -5375, -5479, -6271, -7484, -8733, -9820,-10724,-11420,-11799,-11767,-11380,-10872,-10550,-10563,-10822,-11144,-11396,-11502,-11435,-11260,-11089,-10959,-10817,-10613,-10331, -9984, -9603, },
	/* LAT: -20 */ {  -7373, -6925, -6501, -6071, -5627, -5199, -4812, -4404, -3842, -3160, -2726, -3016, -4158, -5785, -7409, -8763, -9812,-10571,-10989,-11002,-10621, -9999, -9446, -9214, -9270, -9434, -9571, -9582, -9406, -9135, -8947, -8869, -8774, -8570, -8254, -7839, -7373, },
	/* LAT: -10 */ {  -4419, -3871, -3409, -2968, -2510, -2061, -1652, -1207,  -588,   108,   452,    -9, -1373, -3309, -5277, -6861, -7931, -8543, -8790, -8710, -8283, -7586, -6924, -6589, -6555, -6649, -6763, -6782, -6591, -6297, -6151, -6175, -6149, -5939, -5556, -5023, -4419, },
	/* LAT:   0 */ {   -912,  -274,   201,   608,  1028,  1443,  1828,  2260,  2833,  3400,  3584,  3058,  1730,  -201, -2239, -3861, -4838, -5242, -5284, -5101, -4644, -3915, -3210, -2847, -2790, -2857, -2979, -3049, -2918, -2686, -2642, -2805, -2886, -2708, -2283, -1647,  -912, },
	/* LAT:  10 */ {   2556,  3195,  3638,  3983,  4340,  4706,  5052,  5429,  5868,  6227,  6248,  5738,  4631,  3049,  1362,    10,  -767,  -991,  -885,  -640,  -216,   433,  1066,  1398,  1461,  1423,  1325,  1234,  1276,  1375,  1288,  1003,   792,   856,  1209,  1819,  2556, },
	/* LAT:  20 */ {   5413,  5950,  6335,  6633,  6948,  7290,  7628,  7967,  8291,  8482,  8383,  7906,  7056,  5952,  4830,  3936,  3426,  3325,  3487,  3737,  4075,  4547,  5008,  5261,  5320,  5309,  5264,  5206,  5191,  5168,  4985,  4639,  4328,  4232,  4405,  4833,  5413, },
	/* LAT:  30 */ {   7567,  7945,  8265,  8549,  8858,  9204,  9558,  9894, 10163, 10270, 10117,  9687,  9052,  8339,  7682,  7180,  6899,  6869,  7019,  7235,  7486,  7790,  8080,  8253,  8312,  8330,  8336,  8326,  8300,  8213,  7984,  7623,  7261,  7037,  7022,  7219,  7567, },
	/* LAT:  40 */ {   9266,  9487,  9744, 10030, 10356, 10715, 11080, 11415, 11662, 11739, 11584, 11221, 10747, 10275,  9883,  9605,  9462,  9465,  9581,  9746,  9925, 10112, 10285, 10410, 10489, 10553, 10611, 10644, 10625, 10510, 10263,  9909,  9538,  9252,  9109,  9122,  9266, },
	/* LAT:  50 */ {  10802, 10922, 11123, 11392, 11714, 12066, 12418, 12731, 12950, 13006, 12864, 12566, 12203, 11860, 11588, 11405, 11316, 11317, 11387, 11493, 11610, 11728, 11845, 11961, 12081, 12205, 12321, 12393, 12380, 12251, 12002, 11672, 11331, 11046, 10857, 10776, 10802, },
	/* LAT:  60 */ {  12320, 12391, 12539, 12755, 13023, 13321, 13621, 13886, 14060, 14087, 13952, 13705, 13418, 13149, 12930, 12776, 12688, 12659, 12677, 12727, 12797, 12884, 12991, 13125, 13285, 13462, 13627, 13735, 13737, 13613, 13386, 13107, 12829, 12593, 12423, 12330, 12320, },
	/* LAT:  70 */ {  13757, 13797, 13890, 14029, 14206, 14407, 14613, 14793, 14899, 14882, 14749, 14550, 14335, 14132, 13960, 13826, 13733, 13680, 13662, 13676, 13719, 13793, 13897, 14035, 14203, 14391, 14575, 14714, 14757, 14682, 14520, 14321, 14127, 13962, 13841, 13772, 13757, }, // WARNING! black out zone
	/* LAT:  80 */ {  14992, 15002, 15038, 15095, 15169, 15252, 15329, 15375, 15362, 15289, 15181, 15059, 14938, 14825, 14726, 14645, 14583, 14543, 14525, 14529, 14557, 14606, 14679, 14772, 14884, 15012, 15148, 15281, 15388, 15431, 15388, 15298, 15201, 15115, 15049, 15007, 14992, }, // WARNING! black out zone
	/* LAT:  90 */ {  15399, 15399, 15399, 15399, 15399, 15399, 15399, 15399, 15399, 15399, 15399, 15399, 15399, 15399, 15399, 15399, 15399, 15399, 15399, 15399, 15399, 15399, 15399, 15399, 15399, 15399, 15399, 15399, 15399, 15399, 15399, 15399, 15399, 15399, 15399, 15399, 15399, }, // WARNING! black out zone
};
static constexpr float WMM_INCLINATION_MIN_RAD = -1.526; // latitude: -60, longitude: 130
static constexpr float WMM_INCLINATION_MAX_RAD = 1.543; // latitude: 80, longitude: 110


// Magnetic strength data in milli-Gauss * 10
// Model: WMM-2020,
// Version: 0.5.1.11,
// Date: 2023.8192,
static constexpr const int16_t strength_table[19][37] {
	//    LONGITUDE:  -180, -170, -160, -150, -140, -130, -120, -110, -100,  -90,  -80,  -70,  -60,  -50,  -40,  -30,  -20,  -10,    0,   10,   20,   30,   40,   50,   60,   70,   80,   90,  100,  110,  120,  130,  140,  150,  160,  170,  180,
	/* LAT: -90 */ {  5444, 5444, 5444, 5444, 5444, 5444, 5444, 5444, 5444, 5444, 5444, 5444, 5444, 5444, 5444, 5444, 5444, 5444, 5444, 5444, 5444, 5444, 5444, 5444, 5444, 5444, 5444, 5444, 5444, 5444, 5444, 5444, 5444, 5444, 5444, 5444, 5444, },
	/* LAT: -80 */ {  6050, 5986, 5907, 5814, 5711, 5599, 5480, 5358, 5236, 5116, 5002, 4897, 4804, 4724, 4661, 4617, 4593, 4591, 4613, 4660, 4731, 4826, 4941, 5073, 5217, 5367, 5517, 5661, 5792, 5905, 5998, 6066, 6110, 6129, 6124, 6097, 6050, },
	/* LAT: -70 */ {  6295, 6161, 6009, 5843, 5664, 5471, 5266, 5051, 4832, 4617, 4415, 4233, 4076, 3945, 3842, 3766, 3719, 3707, 3737, 3814, 3943, 4125, 4354, 4623, 4919, 5227, 5531, 5815, 6064, 6268, 6418, 6513, 6553, 6545, 6494, 6408, 6295, },
	/* LAT: -60 */ {  6181, 5987, 5784, 5574, 5354, 5119, 4862, 4584, 4292, 4003, 3737, 3510, 3332, 3198, 3100, 3026, 2976, 2956, 2983, 3075, 3245, 3499, 3828, 4216, 4639, 5074, 5495, 5879, 6203, 6450, 6613, 6690, 6691, 6626, 6511, 6359, 6181, },
	/* LAT: -50 */ {  5839, 5608, 5374, 5142, 4909, 4663, 4392, 4089, 3760, 3429, 3128, 2890, 2730, 2640, 2591, 2556, 2521, 2494, 2500, 2574, 2751, 3045, 3444, 3918, 4426, 4932, 5406, 5824, 6162, 6403, 6539, 6579, 6535, 6423, 6260, 6061, 5839, },
	/* LAT: -40 */ {  5390, 5143, 4896, 4656, 4420, 4183, 3927, 3643, 3329, 3003, 2706, 2485, 2370, 2345, 2364, 2384, 2387, 2373, 2359, 2389, 2524, 2805, 3228, 3746, 4294, 4817, 5281, 5668, 5959, 6145, 6232, 6234, 6163, 6031, 5849, 5630, 5390, },
	/* LAT: -30 */ {  4876, 4634, 4394, 4158, 3931, 3712, 3493, 3261, 3005, 2730, 2472, 2292, 2224, 2251, 2318, 2388, 2452, 2499, 2519, 2530, 2601, 2807, 3176, 3669, 4202, 4697, 5112, 5426, 5627, 5726, 5752, 5724, 5644, 5511, 5331, 5114, 4876, },
	/* LAT: -20 */ {  4320, 4106, 3896, 3690, 3494, 3311, 3142, 2978, 2800, 2600, 2410, 2278, 2240, 2286, 2376, 2487, 2614, 2740, 2824, 2858, 2884, 2986, 3233, 3622, 4074, 4500, 4845, 5076, 5179, 5186, 5156, 5109, 5027, 4899, 4732, 4534, 4320, },
	/* LAT: -10 */ {  3790, 3627, 3473, 3326, 3190, 3070, 2966, 2873, 2775, 2659, 2537, 2439, 2396, 2424, 2513, 2643, 2798, 2955, 3075, 3135, 3148, 3178, 3307, 3564, 3892, 4214, 4476, 4636, 4669, 4616, 4548, 4485, 4397, 4272, 4123, 3958, 3790, },
	/* LAT:   0 */ {  3412, 3318, 3233, 3159, 3104, 3065, 3038, 3018, 2992, 2943, 2863, 2770, 2692, 2664, 2710, 2814, 2947, 3081, 3194, 3268, 3298, 3320, 3398, 3559, 3771, 3985, 4163, 4267, 4271, 4203, 4114, 4023, 3911, 3780, 3645, 3521, 3412, },
	/* LAT:  10 */ {  3282, 3250, 3229, 3225, 3248, 3295, 3349, 3401, 3434, 3423, 3354, 3240, 3115, 3023, 3000, 3044, 3126, 3225, 3325, 3409, 3472, 3535, 3625, 3745, 3882, 4022, 4141, 4211, 4211, 4148, 4038, 3894, 3732, 3573, 3437, 3339, 3282, },
	/* LAT:  20 */ {  3399, 3401, 3425, 3478, 3569, 3689, 3816, 3932, 4011, 4023, 3949, 3808, 3642, 3506, 3433, 3423, 3460, 3534, 3632, 3729, 3820, 3920, 4032, 4144, 4254, 4367, 4470, 4535, 4544, 4484, 4346, 4143, 3914, 3704, 3541, 3439, 3399, },
	/* LAT:  30 */ {  3722, 3727, 3779, 3877, 4019, 4188, 4363, 4518, 4625, 4651, 4578, 4423, 4237, 4075, 3973, 3929, 3935, 3988, 4076, 4175, 4274, 4380, 4497, 4615, 4735, 4864, 4985, 5071, 5096, 5036, 4879, 4638, 4363, 4108, 3908, 3779, 3722, },
	/* LAT:  40 */ {  4222, 4217, 4280, 4401, 4566, 4752, 4934, 5091, 5195, 5221, 5153, 5006, 4821, 4650, 4524, 4451, 4427, 4451, 4513, 4594, 4681, 4780, 4895, 5028, 5180, 5343, 5495, 5604, 5641, 5585, 5428, 5189, 4915, 4656, 4445, 4298, 4222, },
	/* LAT:  50 */ {  4832, 4821, 4873, 4981, 5125, 5284, 5433, 5555, 5629, 5639, 5577, 5452, 5291, 5129, 4994, 4898, 4845, 4835, 4861, 4912, 4983, 5074, 5194, 5344, 5520, 5705, 5871, 5987, 6029, 5983, 5851, 5655, 5432, 5217, 5037, 4906, 4832, },
	/* LAT:  60 */ {  5393, 5376, 5401, 5461, 5545, 5639, 5726, 5794, 5828, 5821, 5769, 5676, 5557, 5430, 5312, 5217, 5152, 5119, 5118, 5147, 5204, 5291, 5410, 5557, 5724, 5892, 6038, 6140, 6183, 6159, 6076, 5951, 5806, 5664, 5542, 5449, 5393, },
	/* LAT:  70 */ {  5726, 5703, 5698, 5708, 5729, 5755, 5779, 5795, 5796, 5778, 5741, 5685, 5616, 5541, 5468, 5405, 5357, 5330, 5326, 5346, 5392, 5462, 5554, 5663, 5779, 5893, 5992, 6064, 6102, 6105, 6076, 6023, 5956, 5886, 5820, 5766, 5726, },
	/* LAT:  80 */ {  5790, 5772, 5756, 5744, 5733, 5723, 5714, 5702, 5688, 5670, 5649, 5625, 5598, 5572, 5548, 5528, 5515, 5510, 5516, 5531, 5557, 5592, 5635, 5683, 5733, 5781, 5823, 5857, 5881, 5894, 5896, 5888, 5874, 5855, 5833, 5811, 5790, },
	/* LAT:  90 */ {  5685, 5685, 5685, 5685, 5685, 5685, 5685, 5685, 5685, 5685, 5685, 5685, 5685, 5685, 5685, 5685, 5685, 5685, 5685, 5685, 5685, 5685, 5685, 5685, 5685, 5685, 5685, 5685, 5685, 5685, 5685, 5685, 5685, 5685, 5685, 5685, 5685, },
};
static constexpr float WMM_STRENGTH_MIN_GS = 22.2; // latitude: -30, longitude: -60
static constexpr float WMM_STRENGTH_MAX_GS = 66.9; // latitude: -60, longitude: 140
static constexpr float WMM_STRENGTH_MEAN_GS = 46.4;
static constexpr float WMM_STRENGTH_MEDIAN_GS = 48.8;


