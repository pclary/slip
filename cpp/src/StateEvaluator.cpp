#include "StateEvaluator.hpp"
#include <cmath>


const mjtNum StateEvaluator::w0[] = {
    -0.02038,  0.34462,  0.16041, -0.48774, -0.20436,  0.22043,  0.21103,
    -0.30176, -0.29790,  0.24142, -0.08581, -0.02343, -0.03308, -0.09423,
    -0.05251,  0.03250, -0.00024,  0.10762, -0.24776,  0.13983, -0.18658,
     0.30813, -0.00882, -0.03208,  0.06375,  0.30625, -0.27592,  0.26154,
    -0.24680,  0.16784,  0.24333,  0.07292,  0.18160,  0.08826, -0.28218,
     0.11913,  0.10979, -0.05279,  0.12584, -0.15504,  0.25247,  0.01501,
     0.01240, -0.10587,  0.20758,  0.15902,  0.11674, -0.19866,  0.30230,
     0.15029, -0.18483, -0.12144,  0.08072,  0.10908,  0.01636,  0.29339,
     0.12288, -0.18169,  0.13857,  0.04960, -0.30053, -0.22808,  0.15135,
     0.06284, -0.14237,  0.27626,  0.26904, -0.30063,  0.16341, -0.03925,
    -0.02750, -0.28269, -0.12604, -0.30938, -0.23495,  0.07260, -0.29341,
    -0.11575,  0.04613,  0.06731, -0.00548, -0.20095, -0.22487,  0.07489,
    -0.08091,  0.08706, -0.28868, -0.03617, -0.15969, -0.16879,  0.07245,
    -0.27057, -0.24477, -0.07873,  0.12652, -0.18608, -0.05693, -0.23974,
    -0.13696, -0.03793, -0.32846,  0.51952,  0.25067, -0.06012, -0.34640,
     0.11393, -0.33989, -0.07673,  0.19825, -0.31690,  0.13483, -0.28732,
     0.12943,  0.11922, -0.03568,  0.12817,  0.03502, -0.22532, -0.01441,
    -0.09843, -0.06567, -0.08986,  0.26316,  0.16572,  0.15562,  0.02995,
     0.21105,  0.18418,  0.00276,  0.02111,  0.00404, -0.15863, -0.18947,
     0.11928,  0.23708,  0.09490, -0.24537,  0.23386,  0.22817,  0.11062,
     0.26071, -0.07064, -0.07608,  0.00793, -0.00645,  0.05416,  0.12219,
    -0.10932, -0.27577, -0.14286, -0.13891, -0.01493,  0.16458,  0.09896,
     0.22485,  0.01079, -0.10235,  0.08338, -0.14708, -0.30439,  0.41031,
    -0.61002, -0.23307,  0.38854, -0.06313,  0.24719,  0.28696, -0.25698,
    -0.04219, -0.23327,  0.16963,  0.04162, -0.10906, -0.16464, -0.14708,
    -0.00483, -0.02210,  0.09426,  0.21196, -0.01281, -0.32349,  0.33564,
     0.08377,  0.03455, -0.15062, -0.17483,  0.17151, -0.01086, -0.17688,
    -0.19425, -0.24120,  0.15785,  0.06701, -0.00765,  0.20474,  0.20946,
    -0.06257, -0.02726, -0.27394,  0.19894,  0.12634,  0.37542, -0.15745,
    -0.32692, -0.11483, -0.31084, -0.13498,  0.14249,  0.25302, -0.01516,
     0.07640, -0.14431,  0.18265, -0.12373, -0.22607, -0.12018, -0.14030,
     0.25332, -0.22543, -0.12571,  0.27115,  0.05226,  0.25630, -0.23820,
     0.04476,  0.36699, -0.07368, -0.14909, -0.19817,  0.00875,  0.26367,
    -0.06799,  0.10761,  0.35938, -0.20928,  0.00313, -0.10004,  0.06455,
    -0.06368,  0.01132,  0.23065, -0.15286, -0.19051, -0.11058,  0.16200,
    -0.28426, -0.15322,  0.34016, -0.24479,  0.16154,  0.27020,  0.03345,
    -0.22276,  0.16323,  0.06772,  0.26606,  0.24337,  0.20888, -0.01455,
    -0.19943,  0.42616, -0.22250, -0.26047,  0.60685,  0.20101, -0.25669,
     0.21888, -0.22593,  0.02714,  0.16670,  0.12888, -0.28948, -0.09526,
    -0.00137, -0.11038,  0.00152,  0.03565,  0.20776,  0.06867, -0.10554,
     0.28928,  0.29576, -0.11770,  0.05049, -0.18410, -0.21920,  0.25201,
     0.06892, -0.17512,  0.01736,  0.03122,  0.28353, -0.29587, -0.20618,
    -0.13500, -0.13374, -0.19025,  0.01533, -0.05546,  0.06360, -0.07184,
    -0.01933,  0.24367, -0.13864, -0.05503, -0.29563,  0.11785, -0.00426,
     0.29648,  0.21282,  0.02898,  0.00102, -0.04849, -0.02616,  0.22137,
     0.31152,  0.09200, -0.26848,  0.15126,  0.16796, -0.14080,  0.25892,
    -0.23797, -0.34132,  0.20660, -0.00771, -0.02427,  0.01547,  0.25438,
     0.07161, -0.00065, -0.26277, -0.09736, -0.30567, -0.09375, -0.11060,
     0.12212, -0.01502, -0.23863,  0.17653, -0.04954,  0.26699, -0.26792,
    -0.03203, -0.14087,  0.04213,  0.20840,  0.17895,  0.07168,  0.30511,
     0.03042,  0.03380,  0.25913, -0.16426,  0.03165,  0.22237,  0.09392,
     0.27621,  0.13806, -0.31209, -0.15690,  0.03885,  0.31846,  0.12789,
     0.15551, -0.21780,  0.09809, -0.22927,  0.28612,  0.13431, -0.05009,
    -0.27976,  0.34013, -0.07453, -0.00877,  0.16224, -0.21964, -0.12198,
     0.18590, -0.06910,  0.18809, -0.08715,  0.31971,  0.08537, -0.26263,
     0.21645,  0.07278,  0.28845, -0.01731,  0.26952, -0.01360, -0.23030,
    -0.25884, -0.26390, -0.05315,  0.13966,  0.19101, -0.07409,  0.13518,
     0.33239, -0.13610,  0.01258,  0.06651, -0.40058,  0.10793, -0.26839,
    -0.16956,  0.02903, -0.02319, -0.17441, -0.04191,  0.16705, -0.14196,
     0.11558, -0.19312,  0.00603,  0.06590,  0.34097,  0.31566,  0.09279,
     0.14528, -0.21646,  0.02200,  0.46556,  0.10200, -0.03458, -0.32307,
     0.07576,  0.15076, -0.18111, -0.16464,  0.16210, -0.19073, -0.07102,
    -0.05896, -0.13075,  0.31399, -0.04137, -0.14411, -0.07426,  0.25966,
    -0.05293, -0.32245, -0.13753, -0.22568,  0.38341,  0.22353,  0.00173,
    -0.21317,  0.24897,  0.09095, -0.01395,  0.29216,  0.18507, -0.24260,
    -0.21684, -0.22963, -0.18824,  0.16575,  0.18093,  0.25892, -0.16478,
     0.22980,  0.16093, -0.03374, -0.12729,  0.09158, -0.20294,  0.12074,
     0.09311,  0.33805,  0.11856,  0.15282, -0.21780,  0.05166,  0.01214,
    -0.09245,  0.30315,  0.12412, -0.32317,  0.04338, -0.03423,  0.28291,
     0.00512, -0.23084,  0.01446,  0.24124,  0.19288, -0.01085,  0.25330,
     0.07044,  0.00647,  0.05623,  0.20355, -0.21539, -0.23626, -0.26416,
     0.28428, -0.10859,  0.19268, -0.30748,  0.20094, -0.04286,  0.20965,
     0.26964,  0.19213,  0.11198,  0.13000,  0.12519,  0.01008,  0.27256,
     0.08181,  0.28249, -0.18880,  0.19210,  0.08050,  0.05198,  0.03683,
    -0.05486,  0.38472, -0.11774,  0.34125,  0.17946, -0.33252,  0.12057,
    -0.14856,  0.01288,  0.27672,  0.09491, -0.02571,  0.09031,  0.12962,
     0.17431,  0.13016, -0.06581, -0.08525,  0.09442,  0.06886, -0.12842,
     0.22158, -0.03231,  0.23267, -0.17250, -0.15678,  0.13858, -0.30226,
    -0.30938,  0.18277,  0.16505, -0.16588, -0.10559,  0.27906, -0.29804,
    -0.10303,  0.10146, -0.28901,  0.19693,  0.01767,  0.29949,  0.29420,
    -0.24813,  0.12236, -0.02007, -0.23779, -0.12105,  0.43845, -0.13697,
    -0.00541, -0.19315, -0.11536, -0.21508, -0.00665,  0.07034,  0.43432,
     0.01852, -0.25290,  0.29584,  0.15630, -0.08081, -0.35121, -0.29841,
    -0.03144,  0.21060,  0.01144,  0.21116,  0.11539,  0.24236,  0.08826,
     0.20897,  0.34999, -0.01428, -0.01470,  0.04300, -0.30120,  0.14313,
     0.06248, -0.24755,  0.09891,  0.12829, -0.13645, -0.12943, -0.16223,
    -0.10012, -0.15311, -0.17752,  0.28454, -0.10505, -0.02614, -0.04936,
     0.21001,  0.15857,  0.26314, -0.15973,  0.24643, -0.07741,  0.30720,
     0.18254,  0.26734, -0.07051, -0.30134,  0.16229, -0.18830, -0.07600,
     0.13630, -0.06326,  0.33507, -0.05709, -0.20550,  0.33606,  0.10906,
     0.31835, -0.07274, -0.27276, -0.24085,  0.24535, -0.25120,  0.11191,
    -0.18446,  0.29254,  0.24115, -0.23793,  0.23789, -0.23128, -0.06279,
    -0.26582, -0.26280, -0.01431, -0.00735, -0.11437, -0.25389,  0.20176,
     0.23356, -0.00731, -0.21144,  0.28646,  0.16858,  0.23592, -0.14837,
    -0.20371,  0.16173,  0.02942,  0.26148, -0.28137, -0.41937, -0.04107,
    -0.11838, -0.07298, -0.26826,  0.29431, -0.13005,  0.25644,  0.03190,
    -0.05597,  0.01452,  0.12217,  0.08757,  0.32041, -0.02058, -0.26918,
     0.28088,  0.21058,  0.24053, -0.20003, -0.21685,  0.02518, -0.09997,
    -0.05083,  0.23141,  0.30088,  0.05767,  0.05976, -0.16271, -0.15981,
    -0.29916,  0.06669, -0.28369, -0.34868,  0.26552,  0.11581,  0.11413,
    -0.16934,  0.20224, -0.00679, -0.10733,  0.40197,  0.31955, -0.23123,
    -0.14928,  0.27559, -0.25807,  0.05809, -0.05343,  0.18404,  0.42730,
    -0.17978, -0.24377, -0.28107,  0.15086, -0.09469,  0.30002,  0.23260,
     0.01531, -0.09442,  0.01173, -0.26836, -0.26061, -0.06123,  0.13976,
    -0.17742, -0.10908,  0.24000,  0.14305, -0.09027,  0.10195, -0.03750,
    -0.26924, -0.16280, -0.31087, -0.23271,  0.10756, -0.20787, -0.25516,
    -0.16919,  0.04763,  0.04964,  0.28425, -0.23543, -0.16178,  0.09650,
    -0.10112, -0.01979, -0.10829,  0.30202, -0.01173, -0.30287, -0.28177,
     0.23651, -0.02567, -0.27287, -0.07227,  0.14340, -0.10544,  0.22827,
    -0.28155,  0.00601,  0.06453, -0.23550,  0.27427,  0.25701, -0.15944,
     0.04512, -0.25794,  0.02359,  0.30708,  0.20927,  0.22456, -0.31028,
     0.12230, -0.28711, -0.10488,  0.20535,  0.30766,  0.14754,  0.19732,
    -0.12826, -0.29168, -0.22595,  0.28487,  0.02398, -0.19455,  0.03416,
     0.00786,  0.01579, -0.31692,  0.09038,  0.02715,  0.14711,  0.10070,
     0.05394,  0.25728
};


const mjtNum StateEvaluator::w1[] = {
    -0.11174,  0.25610,  0.04900,  0.22277,  0.04707,  0.02020,  0.09352,
     0.11025, -0.09207,  0.08088,  0.25198,  0.02384, -0.25085,  0.27685,
     0.25870,  0.06504,  0.09343,  0.09222, -0.24439, -0.18044,  0.08398,
     0.20153,  0.15681,  0.27218, -0.06851, -0.23371,  0.18749, -0.01434,
     0.04647, -0.05407, -0.17925,  0.26769, -0.25009, -0.03858,  0.13813,
    -0.24395,  0.09894, -0.04297,  0.21727, -0.05588,  0.23701, -0.14561,
     0.23924,  0.11728,  0.02971,  0.09948,  0.15279, -0.22941, -0.13068,
     0.17956, -0.13112,  0.10071,  0.19140, -0.14819,  0.18191,  0.13614,
    -0.09727, -0.18085, -0.28494, -0.25493,  0.07753, -0.07691,  0.19584,
    -0.01800,  0.17903,  0.02508,  0.20931,  0.03811,  0.02941,  0.05958,
     0.03279, -0.14470,  0.16380, -0.18172, -0.09416, -0.23034, -0.23662,
    -0.01065, -0.13892,  0.12428, -0.00306, -0.09139,  0.13992,  0.25561,
    -0.17225,  0.20287,  0.22417,  0.24766, -0.19570, -0.06089,  0.00828,
    -0.22182,  0.17241, -0.19007,  0.19157, -0.02467, -0.00881, -0.26244,
    -0.26268,  0.00952,  0.05378,  0.11700, -0.15915,  0.22626,  0.22751,
    -0.00927,  0.13722,  0.12198,  0.06754,  0.08851,  0.19806,  0.16346,
    -0.18681, -0.07879,  0.05063,  0.16114, -0.25133, -0.21410, -0.28206,
    -0.03672, -0.18183, -0.26488,  0.01146,  0.10704,  0.21322, -0.07529,
    -0.07715, -0.11279,  0.07042, -0.25401, -0.16248,  0.18322, -0.01003,
    -0.03615,  0.01394,  0.12485,  0.13912,  0.16535, -0.13687, -0.18618,
    -0.14864,  0.16255,  0.11304,  0.21486,  0.00227, -0.10084,  0.02080,
     0.03274, -0.24263, -0.10491,  0.00770,  0.12725, -0.11476,  0.11861,
    -0.27064,  0.21551, -0.16799,  0.01124, -0.22367,  0.04450,  0.05195,
    -0.25560,  0.02596,  0.21709,  0.13558, -0.24375,  0.08582, -0.02494,
     0.12177,  0.04274, -0.02022, -0.02604,  0.02641,  0.16910,  0.26060,
     0.17977, -0.15937,  0.11726, -0.03221,  0.08478,  0.25528, -0.06780,
     0.13777, -0.09378,  0.02703,  0.20722, -0.00170,  0.13894,  0.20867,
     0.05889, -0.20734, -0.26882,  0.08824, -0.19544, -0.24797,  0.11805,
    -0.06152, -0.26582, -0.22434, -0.11113, -0.22959, -0.19963,  0.04424,
     0.06139, -0.12339, -0.28932,  0.27895,  0.18673, -0.18467, -0.24513,
    -0.26196,  0.18294,  0.01026, -0.21984,  0.05265, -0.25387, -0.01593,
     0.14120,  0.02204, -0.13598, -0.06501, -0.25836,  0.16351, -0.14026,
     0.01242,  0.08433,  0.09549,  0.20539,  0.04874, -0.07993,  0.17926,
     0.01945,  0.19767, -0.00773, -0.25664,  0.16581, -0.25524,  0.13904,
     0.28903, -0.15533, -0.25706,  0.27785, -0.19109, -0.18454, -0.02745,
     0.24192,  0.13424,  0.03859, -0.20416, -0.06402, -0.16862, -0.18805,
     0.12348,  0.04937,  0.06091,  0.19364,  0.09890, -0.19879,  0.13765,
     0.23988,  0.19056,  0.13790,  0.27467,  0.21192, -0.10285,  0.24920,
    -0.23407,  0.18130,  0.17112, -0.26094, -0.00762, -0.09923,  0.13984,
    -0.20100,  0.22751, -0.23286, -0.11698,  0.10915,  0.21953,  0.07226,
    -0.10348, -0.21292,  0.09226, -0.10081, -0.19037,  0.15233, -0.21664,
    -0.20844, -0.03942, -0.14754, -0.19748,  0.21832, -0.08330, -0.08740,
    -0.06158,  0.23885,  0.12648, -0.07800,  0.26212, -0.13478, -0.23393,
     0.13850, -0.20779, -0.15702,  0.11979,  0.20502,  0.09606, -0.00418,
     0.17923,  0.20735, -0.22178, -0.00220,  0.05922, -0.13827,  0.01425,
    -0.06152, -0.14774, -0.22757, -0.08643, -0.15917, -0.01784, -0.18006,
     0.10823, -0.21248,  0.13989,  0.21295,  0.05926,  0.05723,  0.18120,
    -0.05875,  0.00072, -0.18339,  0.08385,  0.22931,  0.04266,  0.07561,
     0.05157, -0.04918,  0.22860, -0.20163, -0.24609,  0.02916,  0.08259,
    -0.04068,  0.24593,  0.04201, -0.03442,  0.03361, -0.05942,  0.16883,
     0.08350,  0.02488,  0.13454,  0.11903,  0.13851, -0.00709,  0.09845,
    -0.07649,  0.12631, -0.08694, -0.22348, -0.05678,  0.19640, -0.07389,
     0.17948, -0.06624,  0.01383, -0.21259, -0.04921, -0.03328,  0.01395,
    -0.04581, -0.03378, -0.06517, -0.15161,  0.06298, -0.06955, -0.17576,
    -0.22294, -0.15776, -0.20482,  0.00361, -0.01540, -0.05438,  0.19562,
     0.18281,  0.21554, -0.00564, -0.23132,  0.03011, -0.17499,  0.03865,
    -0.25135,  0.24045,  0.03614,  0.02766,  0.00652,  0.01927,  0.03655,
    -0.01339,  0.02707,  0.10740,  0.13514, -0.11891,  0.23830, -0.17357,
    -0.19023, -0.06804,  0.39664,  0.08826,  0.03414,  0.09900,  0.19894,
     0.13975,  0.21963, -0.08589, -0.12175, -0.23152, -0.13122,  0.05640,
     0.15285, -0.01213,  0.29126, -0.17733,  0.27166,  0.11359, -0.08445,
    -0.06731, -0.01983, -0.08922,  0.22932,  0.08299,  0.16516,  0.29512,
     0.15968, -0.30815,  0.23129,  0.17083, -0.08645, -0.05884,  0.00947,
     0.08691,  0.09876,  0.04750, -0.13618,  0.09864, -0.03571,  0.18072,
    -0.16631, -0.15671,  0.09504, -0.04550,  0.00161,  0.08468, -0.20157,
    -0.21408,  0.07617, -0.10410, -0.21867,  0.17532, -0.17796,  0.22039,
    -0.00871,  0.11870, -0.26572, -0.13872,  0.26456,  0.19886,  0.01760,
    -0.12341,  0.20808,  0.15846,  0.17008,  0.00982, -0.08948, -0.19555,
    -0.12535, -0.04927,  0.11130, -0.26103,  0.03495, -0.01059, -0.22661,
     0.06081,  0.18040,  0.28628,  0.22761, -0.02642, -0.13842,  0.30499,
    -0.14010, -0.12949, -0.24410,  0.31791,  0.25459, -0.19551,  0.30857,
    -0.03618, -0.04656,  0.22449,  0.20283,  0.14231,  0.19003, -0.08959,
     0.21677, -0.09626,  0.16123,  0.19972,  0.29344,  0.20614, -0.07969,
    -0.17247, -0.02285,  0.17543, -0.01036, -0.08631,  0.23651,  0.14827,
    -0.23030,  0.11378, -0.14624, -0.01278,  0.15177,  0.13026, -0.06066,
     0.00191,  0.07971, -0.21014,  0.00102, -0.00146, -0.10841,  0.21457,
    -0.01576, -0.16633, -0.03121,  0.17986, -0.24683, -0.15514,  0.00628,
     0.11832, -0.23088,  0.17644, -0.06946,  0.21603, -0.14298,  0.07563,
     0.26379,  0.04937,  0.14616, -0.14577, -0.07956, -0.13705, -0.21646,
     0.20365,  0.09392, -0.18664,  0.05792, -0.23992, -0.10462,  0.07002,
    -0.08729, -0.09214,  0.13791,  0.24790,  0.20921, -0.08046, -0.19541,
    -0.25468,  0.18277,  0.20072,  0.05594,  0.02464,  0.18866, -0.04605,
    -0.20103,  0.21921, -0.11522,  0.06838, -0.04242,  0.04067, -0.25060,
     0.02660,  0.22081, -0.23520, -0.25166,  0.10917, -0.05804,  0.21027,
    -0.17526, -0.22244, -0.26535,  0.03164,  0.17645, -0.09423, -0.26080,
     0.20506, -0.00084,  0.21250, -0.05820,  0.22378, -0.14471,  0.07665,
     0.17935, -0.03992,  0.19605,  0.12271,  0.08907, -0.11002,  0.05182,
     0.05000,  0.12029, -0.07357, -0.02833,  0.03187,  0.12536, -0.26596,
     0.25145, -0.21291, -0.12963,  0.05745,  0.12974, -0.04620, -0.24168,
    -0.19385, -0.10912, -0.00174, -0.25847,  0.13499, -0.27365, -0.03867,
     0.23656, -0.27369,  0.07943,  0.07332, -0.15470,  0.03152,  0.03410,
    -0.10910,  0.16948, -0.25585, -0.23038,  0.20727,  0.05943,  0.14267,
    -0.18431,  0.18335,  0.14744,  0.11289,  0.32573, -0.16072, -0.09144,
    -0.04360, -0.04658,  0.16917,  0.13529,  0.00976, -0.09682,  0.12267,
    -0.17230, -0.13885,  0.06705, -0.17037,  0.04206,  0.19829, -0.00649,
     0.19595, -0.23829, -0.02685, -0.11156, -0.21021,  0.14641,  0.21538,
    -0.05159, -0.06875, -0.09376, -0.13871, -0.14851,  0.23181, -0.08681,
    -0.16348, -0.22667, -0.10328,  0.01119,  0.17853,  0.21267, -0.22174,
     0.09905,  0.24526, -0.21248,  0.22630,  0.08000,  0.05523, -0.27169,
    -0.13274, -0.12572, -0.10176, -0.27566,  0.10115,  0.17984, -0.26063,
     0.21969, -0.12249,  0.15214,  0.17750, -0.05320,  0.24489, -0.02561,
    -0.22282,  0.08524,  0.12602,  0.19477, -0.26272,  0.17327,  0.20234,
    -0.03654,  0.19474, -0.03429, -0.04380, -0.00811,  0.26384, -0.28687,
     0.16004, -0.02305, -0.15810,  0.03004,  0.10487,  0.11375,  0.24093,
     0.11023,  0.02119, -0.27565, -0.09433,  0.00639, -0.16042,  0.07860,
     0.04741, -0.12796, -0.07254,  0.10357, -0.00617,  0.20177, -0.07448,
     0.23218,  0.13542, -0.03814, -0.00276, -0.06762, -0.15855,  0.26084,
    -0.28797, -0.01381, -0.25522,  0.22233, -0.12120,  0.18318,  0.08472,
     0.24867, -0.06718,  0.01272, -0.08687,  0.32225, -0.19441,  0.15175,
    -0.06181,  0.25083,  0.29968,  0.09926,  0.19121,  0.35168,  0.23002,
     0.19282,  0.15483, -0.14027,  0.33534, -0.16983,  0.06108,  0.06547,
     0.01822, -0.15046, -0.06732,  0.19653,  0.37870,  0.13616,  0.23888,
    -0.20084, -0.11083,  0.23346,  0.17464, -0.20018,  0.14544,  0.13066,
    -0.15379,  0.25825,  0.11488,  0.04059, -0.03290, -0.21715,  0.02012,
     0.16694, -0.23963,  0.15153, -0.09689,  0.16983,  0.00709,  0.18337,
     0.19985,  0.11040, -0.01707,  0.03653, -0.04529, -0.14974, -0.25565,
    -0.25591,  0.13348, -0.05760,  0.00398,  0.02527,  0.18539,  0.05393,
    -0.10580, -0.08830,  0.18646, -0.08234, -0.14091,  0.18579,  0.00414,
     0.01537, -0.11465,  0.12096,  0.21160,  0.14143, -0.12079,  0.27393,
     0.28013,  0.23274,  0.22328, -0.14446,  0.16750, -0.16990, -0.22055,
    -0.19598,  0.13892, -0.05240,  0.09086,  0.05692,  0.12676,  0.16768,
    -0.09130, -0.01865,  0.14075, -0.05086,  0.14925, -0.06167, -0.18569,
    -0.18097, -0.15552, -0.09678, -0.20449, -0.05728, -0.24153,  0.20100,
    -0.08099, -0.17006,  0.01512, -0.22834,  0.23769,  0.14737, -0.10637,
     0.18359,  0.03499,  0.10174, -0.07466,  0.07638, -0.04603, -0.15732,
     0.24548, -0.09025,  0.11764,  0.27592,  0.13076,  0.23321, -0.05457,
     0.25257,  0.18207,  0.12685,  0.03793, -0.16607, -0.24277,  0.16404,
    -0.24271,  0.23954,  0.14662, -0.11721,  0.01948,  0.11116,  0.01594,
     0.04608, -0.00610, -0.22911, -0.00752,  0.08874, -0.20299,  0.06339,
    -0.04602, -0.21218,  0.08762,  0.02810,  0.12985,  0.18185,  0.26511,
     0.10221, -0.16119,  0.04542, -0.24171,  0.03831,  0.09986, -0.21470,
     0.20006,  0.20103, -0.02610, -0.01298, -0.12704,  0.11038, -0.02412,
     0.08991, -0.14969, -0.16353,  0.19559, -0.00858, -0.09080, -0.09871,
     0.12056,  0.15765,  0.10550,  0.03149, -0.21026,  0.00332,  0.14014,
     0.12576,  0.00244, -0.20039,  0.11830,  0.24381, -0.09328, -0.08066,
     0.27873,  0.19560,  0.23568,  0.24995,  0.24186,  0.16318, -0.08382,
     0.09921, -0.21891, -0.26739, -0.18267, -0.08775, -0.14002,  0.16042,
     0.02085, -0.20297,  0.16204,  0.14049, -0.02158, -0.06116,  0.10074,
    -0.20041, -0.23719,  0.25712, -0.04200, -0.13605, -0.26634,  0.11267,
     0.07442,  0.26689,  0.19311,  0.20519,  0.03493,  0.13493, -0.05125,
     0.22439, -0.00008, -0.16990,  0.11923, -0.20827, -0.28146,  0.07506,
    -0.06562,  0.20291, -0.02367,  0.20434, -0.09744, -0.10129, -0.04432,
     0.12186,  0.10385, -0.09072, -0.09246, -0.01896, -0.16397, -0.13641,
     0.12404,  0.20394, -0.26796,  0.22142, -0.17766,  0.18241,  0.26637,
    -0.29440,  0.07562,  0.05683,  0.36456,  0.12944, -0.12584,  0.07084,
    -0.12928, -0.23936, -0.08504,  0.07400, -0.14220,  0.18977, -0.26235,
     0.09286,  0.03907,  0.07233,  0.06377,  0.20495, -0.08724,  0.33391,
    -0.15444,  0.17065,  0.03699, -0.22179, -0.36845, -0.05991,  0.30896,
    -0.05839,  0.11419,  0.09109,  0.07587,  0.33590,  0.03892, -0.11164,
     0.22361, -0.16770, -0.11088,  0.08577, -0.00998,  0.17986,  0.30145,
    -0.16358, -0.13668,  0.21095, -0.23733, -0.05830,  0.21839, -0.09429,
    -0.22522,  0.11348,  0.05438,  0.27049,  0.03180,  0.15116, -0.18169,
     0.01193,  0.13524, -0.04286, -0.15635,  0.15202,  0.07660,  0.13585,
    -0.12805,  0.19627,  0.32220, -0.10147, -0.22611,  0.01641, -0.18481,
     0.15456, -0.16757,  0.22983,  0.28015, -0.15775,  0.07561, -0.09349,
     0.22577,  0.08666,  0.19487, -0.15844,  0.07797,  0.00213, -0.17785,
    -0.19995,  0.25301,  0.15266, -0.18197, -0.04820,  0.20532,  0.10387,
     0.28707,  0.22212,  0.10171, -0.14532,  0.02696,  0.16546,  0.13040,
    -0.15140, -0.03936, -0.21251,  0.09963,  0.25871,  0.21758,  0.10795,
     0.12043,  0.11278, -0.17010, -0.05577, -0.01374, -0.01118,  0.15416,
     0.00715, -0.13666, -0.16400,  0.00762, -0.06797, -0.08231, -0.02785,
     0.15366,  0.00001,  0.21609, -0.24766,  0.15220,  0.07521, -0.13187,
     0.18662,  0.06776,  0.21367,  0.18433,  0.06896,  0.13175, -0.01411,
    -0.27101, -0.07186, -0.03309, -0.08010, -0.17450,  0.06184,  0.03600,
    -0.27646, -0.02976,  0.15665, -0.12469, -0.16670, -0.23953, -0.20212,
    -0.15658,  0.24730,  0.01822,  0.21501,  0.04713, -0.20428,  0.26684,
     0.09239,  0.11317,  0.06615, -0.34156, -0.13588, -0.10783,  0.00048,
     0.08405, -0.28365,  0.09262, -0.18443, -0.35677,  0.23018,  0.18775,
    -0.07435,  0.02750,  0.11994, -0.10884, -0.22623, -0.31392,  0.30543,
     0.13573, -0.21555,  0.26419, -0.13067, -0.08041,  0.32472, -0.02847,
    -0.03503, -0.07935,  0.20081, -0.24325, -0.05671,  0.34906, -0.29836,
     0.11117,  0.07828,  0.07083, -0.18583,  0.11443, -0.05575, -0.22310,
    -0.14709,  0.19884,  0.18886,  0.16167,  0.05197, -0.19257, -0.02250,
    -0.12298,  0.02332,  0.06780,  0.01971,  0.18242,  0.09050, -0.19129,
     0.16453,  0.12245, -0.19940, -0.18872,  0.20017,  0.19441, -0.08156,
    -0.13946, -0.00861,  0.28707,  0.18111, -0.08790,  0.27436,  0.15210,
     0.18406,  0.05418, -0.18784, -0.03148,  0.22337,  0.26798,  0.16951,
    -0.02963, -0.03416, -0.11930, -0.13846,  0.14192, -0.09625,  0.10285,
     0.21004,  0.20983,  0.32176,  0.17992, -0.23678,  0.01214, -0.09209,
    -0.15100,  0.16779,  0.15083,  0.04610,  0.28288, -0.12250,  0.12567,
     0.10948, -0.13160,  0.02480,  0.06933, -0.25926,  0.11416, -0.13532,
     0.11445, -0.00657,  0.13387,  0.24705,  0.23196,  0.18664, -0.06204,
    -0.13441, -0.03584, -0.12639, -0.19239,  0.19214, -0.21133,  0.17806,
    -0.17995, -0.06483, -0.03331, -0.18700, -0.25823, -0.25871,  0.23755,
     0.12954,  0.24502, -0.13063,  0.17230,  0.00654, -0.02371, -0.15620,
     0.12209,  0.03422,  0.11293,  0.04269,  0.02656,  0.20481, -0.22807,
     0.22139, -0.00202,  0.02902,  0.10313, -0.03788,  0.20946, -0.28127,
    -0.22310,  0.13474,  0.26693, -0.10543,  0.09685,  0.05153,  0.03551,
    -0.15710,  0.06886,  0.13336, -0.23248,  0.07681, -0.14884,  0.18329,
     0.17906,  0.04841,  0.27776,  0.23262,  0.22324,  0.04946,  0.25445,
    -0.11753,  0.02192,  0.07481,  0.20272,  0.20894,  0.16125,  0.08451,
    -0.08885,  0.09716, -0.00845,  0.02624, -0.13151,  0.17245, -0.12763,
     0.00556, -0.19101,  0.18589,  0.20591,  0.05564,  0.02442,  0.23064,
     0.11043, -0.16813, -0.13950,  0.21642,  0.01511,  0.13569,  0.08405,
    -0.17951,  0.08688,  0.20433,  0.23449, -0.24474, -0.15644, -0.26186,
     0.14890, -0.24740, -0.25380,  0.10645, -0.00605,  0.13600,  0.22446,
     0.01441, -0.14550, -0.17701, -0.16729, -0.21652, -0.26418, -0.01736,
     0.08406,  0.10316, -0.06485,  0.19145,  0.18707,  0.26007, -0.05074,
     0.08433,  0.13443,  0.18343,  0.11830,  0.15166,  0.22707, -0.10632,
     0.09533,  0.14189, -0.05551, -0.21769,  0.01965,  0.22939,  0.16030,
    -0.03554,  0.12233, -0.24099,  0.25111,  0.18395, -0.12850,  0.00837,
     0.18862,  0.19589,  0.05696,  0.05744,  0.19724, -0.03744, -0.20185,
    -0.26763,  0.26867, -0.03884, -0.13534,  0.25419,  0.14421,  0.27651,
    -0.06464,  0.16177,  0.13348,  0.15195, -0.11746, -0.19711, -0.04252,
     0.09838, -0.07765,  0.12864, -0.07667,  0.19457, -0.06574, -0.22145,
     0.21439,  0.07465,  0.07274,  0.15058,  0.05470, -0.08001, -0.13173,
    -0.17358,  0.11610,  0.25402,  0.14315,  0.19829, -0.20423, -0.11731,
    -0.13923,  0.17230,  0.11033, -0.22955,  0.10729, -0.01049,  0.15463,
    -0.08373,  0.10118,  0.00290,  0.03764, -0.28881, -0.22698,  0.09487,
    -0.02893, -0.00152,  0.12182, -0.26519, -0.16970, -0.10667, -0.13111,
    -0.10407, -0.14019, -0.04010,  0.03421,  0.25341,  0.17122, -0.06887,
    -0.25105, -0.03194,  0.01697,  0.17943,  0.15722, -0.03694, -0.01283,
    -0.04426,  0.12885, -0.03261,  0.26497,  0.37883,  0.00488,  0.23333,
    -0.11710, -0.11074,  0.41771, -0.13873, -0.08556,  0.34498,  0.06694,
     0.18988,  0.14856,  0.24211,  0.10873,  0.21397, -0.16583,  0.16547,
    -0.24125,  0.06266, -0.06901,  0.10014,  0.06147,  0.09522,  0.02699,
     0.05326,  0.02296, -0.13893,  0.05237,  0.26575, -0.16116, -0.03914,
    -0.24295,  0.23884, -0.20684,  0.17732,  0.24075, -0.15274, -0.19385,
     0.06140, -0.04832,  0.11896, -0.08005, -0.02903, -0.01679, -0.09938,
    -0.00329, -0.14469,  0.10300, -0.26319,  0.07034, -0.05124, -0.09173,
     0.02672, -0.02828, -0.17050,  0.17711,  0.23726,  0.12075, -0.18704,
     0.00200,  0.18951, -0.06615, -0.18868, -0.25694,  0.18547,  0.10948,
    -0.08260,  0.15877, -0.26515, -0.24073,  0.16000, -0.15293,  0.24922,
     0.03227,  0.24536,  0.07846, -0.25115,  0.07942,  0.14776, -0.00242,
    -0.18389, -0.11881, -0.11126,  0.04551,  0.04917,  0.22193,  0.06015,
     0.18271, -0.23423, -0.21228,  0.25727,  0.05206,  0.25210,  0.20511,
     0.08006, -0.17427,  0.06249,  0.11356,  0.02943,  0.06966,  0.19415,
     0.02517,  0.19724,  0.05818,  0.26225, -0.10630,  0.04393,  0.32421,
    -0.16734,  0.31152, -0.17120,  0.19830
};


const mjtNum StateEvaluator::w2[] = {
    -0.15052, -0.37898, -0.34807, -0.23822, -0.21586,  0.49568,  0.28454,
     0.00909, -0.42224, -0.20006, -0.19314, -0.39001, -0.12670,  0.09546,
     0.16529, -0.20265, -0.13764,  0.26809,  0.18874, -0.52366, -0.07931,
     0.02702, -0.06148,  0.02768, -0.29275,  0.59381, -0.41861, -0.12233,
    -0.17071,  0.55563, -0.08763, -0.38503,  0.18848, -0.23017, -0.08880,
     0.10365, -0.18849, -0.49027,  0.10252,  0.41501,  0.10777, -0.19169,
     0.12445,  0.14110, -0.29272, -0.24600,  0.21824,  0.16935,  0.04757,
     0.02227,  0.49158, -0.09945,  0.42545,  0.18258,  0.07099, -0.28689,
     0.12251, -0.01845, -0.30974,  0.51564,  0.38785, -0.34899, -0.00636,
    -0.25194, -0.02439, -0.40412,  0.29650,  0.20327,  0.38182, -0.50819,
     0.16203,  0.31293,  0.39565,  0.07036,  0.05104,  0.03738,  0.23909,
     0.57311, -0.31306,  0.15979
};


const mjtNum StateEvaluator::b0[] = {
    -0.00639,  0.05848,  0.03995, -0.02506,  0.00434, -0.17620,  0.00121,
     0.03831, -0.00843,  0.02032, -0.06233,  0.17452,  0.01943,  0.03400,
     0.00542, -0.01998, -0.15104, -0.05601,  0.09480, -0.00766, -0.01998,
     0.10957,  0.08961,  0.01863, -0.10540,  0.11256,  0.01233, -0.00575,
     0.25475, -0.09413,  0.01675,  0.01672,  0.00185,  0.00553, -0.00179,
     0.21105, -0.01673,  0.12841,  0.03968,  0.00011
};


const mjtNum StateEvaluator::b1[] = {
     0.01829, -0.04726, -0.08423, -0.02138,  0.00391,  0.18321,  0.01832,
    -0.01719, -0.05834, -0.04995, -0.03780, -0.02058, -0.04589, -0.00888,
     0.01671, -0.01565, -0.06961,  0.03121,  0.17713, -0.19842, -0.05489,
     0.07433, -0.04815,  0.04624, -0.05363,  0.27994, -0.06114,  0.00976,
    -0.01749,  0.28994, -0.06361, -0.09253, -0.00667,  0.01702, -0.02121,
     0.06683, -0.05715, -0.25186,  0.02003,  0.15203
};


const mjtNum StateEvaluator::b2[] = {
    0.31703, -0.30489
};


double StateEvaluator::value(robot_state_t X,
                             const terrain_t& terrain,
                             goal_t goal)
{
    auto vs = stability(X, terrain);
    auto vg = goal_value(X, goal);
    return combine_value(vs, vg);
}


double StateEvaluator::stability(robot_state_t X,
                                 const terrain_t& terrain)
{
    // Construct input vector
    mjtNum input[] = {
        std::fmod(X.qpos[BODY_THETA] + M_PI, M_2_PI) - M_PI,
        X.qvel[BODY_DX],
        X.qvel[BODY_DY],
        X.qvel[BODY_DTHETA],
        X.qpos[RIGHT_L],
        X.qpos[RIGHT_L_EQ],
        X.qpos[RIGHT_THETA],
        X.qpos[RIGHT_THETA_EQ],
        X.qvel[RIGHT_DL],
        X.qvel[RIGHT_DL_EQ],
        X.qvel[RIGHT_DTHETA],
        X.qvel[RIGHT_DTHETA_EQ],
        X.qpos[LEFT_L],
        X.qpos[LEFT_L_EQ],
        X.qpos[LEFT_THETA],
        X.qpos[LEFT_THETA_EQ],
        X.qvel[LEFT_DL],
        X.qvel[LEFT_DL_EQ],
        X.qvel[LEFT_DTHETA],
        X.qvel[LEFT_DTHETA_EQ],
    };

    // Layer sizes
    const size_t ni  = sizeof input / sizeof input[0];
    const size_t nh0 = sizeof b0 / sizeof b0[0];
    const size_t nh1 = sizeof b1 / sizeof b1[0];
    const size_t no  = sizeof b2 / sizeof b2[0];

    // Initialize layers
    mjtNum h0[nh0];
    mjtNum h1[nh0];
    mjtNum out[no];

    // Hidden layers
    mju_mulMatVec(h0, w0, input, nh0, ni);
    mju_addTo(h0, b0, nh0);
    for (size_t i = 0; i < nh0; ++i)
        h0[i] = std::fmax(h0[i], 0);

    mju_mulMatVec(h1, w1, h0, nh1, nh0);
    mju_addTo(h1, b1, nh1);
    for (size_t i = 0; i < nh1; ++i)
        h1[i] = std::fmax(h1[i], 0);

    // Output layer
    mju_mulMatVec(out, w2, h1, no, nh1);
    mju_addTo(out, b2, no);

    // Softmax
    double sumexp = 0;
    for (auto x : out)
        sumexp += std::exp(x);
    double vs = std::exp(out[0]) / sumexp;

    // Absolute crash check
    if (X.qpos[BODY_Y] || std::fabs(X.qpos[BODY_THETA]) > M_PI_2 ||
        std::fabs(X.qpos[RIGHT_THETA] - X.qpos[LEFT_THETA]) > 0.8 * M_PI)
        vs = 0;

    return vs;
}


double StateEvaluator::goal_value(robot_state_t X, goal_t goal)
{
    // Percent-based velocity error score
    double saturation = 2; // Full error
    double deadband = 1.1; // Less than this (multiplicative)
                           // difference treated as zero error
    double em =
        std::fmax(
            std::fabs(
                std::log(
                    std::fmin(
                        std::fmax(
                            X.qvel[BODY_DX] / goal.dx,
                            1/saturation),
                        saturation)))
            - log(deadband), 0) /
        log(saturation / deadband);

    // Difference-based velocity error score
    saturation = 1;
    deadband = 0.1;
    double ed =
        std::fmin(
            std::fmax(
                std::fabs(X.qvel[BODY_DX] - goal.dx)
                - deadband, 0) /
            (saturation - deadband), 1);

    // Error is the minimum of the two error scores, and score is inverted
    return 1 - std::fmin(em, ed);
}


double StateEvaluator::combine_value(double stability, double goal_value)
{
    return (stability < 0.5) ? stability : (goal_value + 1) / 2;
}