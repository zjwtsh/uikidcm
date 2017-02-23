module(..., package.seeall);

--[[
% Rows of the above matrices indicate the following variables
ind_LHipYawPitch=1;
ind_LHipRoll=2;
ind_LHipPitch=3;
ind_LKneePitch=4;
ind_LAnklePitch=5;
ind_LAnkleRoll=6;
ind_RHipYawPitch=7;
ind_RHipRoll=8;
ind_RHipPitch=9;
ind_RKneePitch=10;
ind_RAnklePitch=11;
ind_RAnkleRoll=12;
-]]

-- Bezier coefficients when leg is in stance
--[[
-- Webots coefficients
alpha_L = { {  0.0116, -0.0434, -0.0938, -0.0815, -0.0716, -0.0137},{  0.0347, -0.1572, -0.1608, -0.1141, -0.1822, -0.0426},{ -0.8720, -0.9939, -0.7674, -0.8201, -0.8512, -0.7867},{  0.3166,  0.7123,  0.4419,  0.5367,  0.7343,  0.7908},{  0.2060, -0.0657, -0.0368, -0.0744, -0.2365, -0.3534},{ -0.0339,  0.1279,  0.2717,  0.2313,  0.2101,  0.0398},{  0.0064, -0.0287, -0.1031, -0.1698, -0.0340, -0.0209},{  0.0175, -0.0830, -0.2796, -0.4446, -0.1010, -0.0569},{ -0.8218, -0.5803,  0.1116, -3.9721,  0.5099, -0.9724},{  0.8857,  0.3302,  0.6159,  4.7666, -2.2055,  0.4532},{ -0.4188, -0.0324, -1.2454, -1.3482,  1.3277,  0.1769},{ -0.0194,  0.0872,  0.2985,  0.4749,  0.1067,  0.0601}
} ;

alpha_R = { { -0.0065,  0.0286,  0.1031,  0.1698,  0.0339,  0.0208},{ -0.0179,  0.0811,  0.2820,  0.4426,  0.1022,  0.0564},{ -0.8232, -0.5830,  0.1051, -3.9613,  0.5055, -0.9778},{  0.8827,  0.3300,  0.6010,  4.7720, -2.1928,  0.4708},{ -0.4143, -0.0315, -1.2179, -1.3593,  1.3204,  0.1645},{  0.0197, -0.0870, -0.2984, -0.4750, -0.1066, -0.0598},{ -0.0117,  0.0433,  0.0939,  0.0815,  0.0716,  0.0136},{ -0.0351,  0.1569,  0.1614,  0.1133,  0.1829,  0.0419},{ -0.8693, -0.9940, -0.7641, -0.8232, -0.8502, -0.7891},{  0.3166,  0.7167,  0.4397,  0.5375,  0.7328,  0.7904},{  0.2031, -0.0694, -0.0357, -0.0750, -0.2347, -0.3507},{  0.0342, -0.1277, -0.2719, -0.2313, -0.2101, -0.0395}
};
--]]

-- For real robot
--[[
alpha_L = {
{   0.0034, -0.0136, -0.0346, -0.0320, -0.0485, -0.0334, -0.0376, -0.0197, -0.0035},{   0.0108, -0.0159, -0.3972,  1.0329, -1.1937,  0.5988, -0.0105, -0.1186, -0.0046},{  -0.8807, -0.8721, -0.8548, -0.8344, -0.8299, -0.8158, -0.8178, -0.8110, -0.7975},{   0.7319,  0.7503,  0.7477,  0.7324,  0.7396,  0.7447,  0.7654,  0.7880,  0.7942},{  -0.2003, -0.2270, -0.2428, -0.2492, -0.2610, -0.2802, -0.2980, -0.3262, -0.3458},{  -0.0099,  0.0397,  0.1013,  0.0932,  0.1412,  0.0972,  0.1098,  0.0577,  0.0102},{   0.0013, -0.0153, -0.0419, -0.0034, -0.1305, -0.0046, -0.0469, -0.0206, -0.0055},{   0.0036, -0.0422, -0.1139, -0.0125, -0.3517, -0.0154, -0.1278, -0.0566, -0.0152},{  -0.7841, -0.9341,  0.0052, -2.1556, -0.5649, -2.2021, -0.3447, -0.9771, -0.8762},{   0.7790,  1.1056, -0.7076,  3.7254, -0.1190,  3.3399, -0.4126,  0.8946,  0.7258},{  -0.3439, -0.5201,  0.3498, -1.9132,  0.3176, -1.4819,  0.4045, -0.2664, -0.1987},{  -0.0041,  0.0449,  0.1223,  0.0107,  0.3776,  0.0141,  0.1370,  0.0600,  0.0162}
};

alpha_R = { {  -0.0009,  0.0137,  0.0441,  0.0022,  0.1299,  0.0066,  0.0450,  0.0212,  0.0056},{  -0.0024,  0.0378,  0.1201,  0.0090,  0.3501,  0.0208,  0.1227,  0.0583,  0.0153},{  -0.7893, -0.9463,  0.0146, -2.1957, -0.5506, -2.2175, -0.3377, -0.9811, -0.8779},{   0.7770,  1.1072, -0.7243,  3.7515, -0.1407,  3.3439, -0.4243,  0.8931,  0.7224},{  -0.3368, -0.5098,  0.3588, -1.9033,  0.3314, -1.4770,  0.4130, -0.2619, -0.1935},{   0.0026, -0.0402, -0.1282, -0.0085, -0.3745, -0.0203, -0.1316, -0.0618, -0.0163},{  -0.0029,  0.0123,  0.0361,  0.0321,  0.0464,  0.0363,  0.0356,  0.0203,  0.0035},{  -0.0093,  0.0106,  0.4070, -1.0345,  1.1805, -0.5794, -0.0020,  0.1209,  0.0047},{  -0.8763, -0.8689, -0.8510, -0.8318, -0.8275, -0.8139, -0.8148, -0.8086, -0.7945},{   0.7377,  0.7526,  0.7537,  0.7290,  0.7475,  0.7410,  0.7694,  0.7882,  0.7949},{  -0.2105, -0.2326, -0.2524, -0.2487, -0.2709, -0.2787, -0.3048, -0.3289, -0.3495},{   0.0085, -0.0364, -0.1051, -0.0937, -0.1350, -0.1053, -0.1042, -0.0592, -0.0103}
};
--]]

-- Alpha from slower walk parameters, longer stride length
--[[
-- Slow walk
walk.tZmp = 0.165;
walk.tStep = 0.5;
walk.phSingle={0.1,0.9};
walk.supportY = 0.010;
walk.supportX = -0.005;
walk.stepHeight = 0.06;
walk.qLArm=math.pi/180*vector.new({90,0,-80});
walk.qRArm=math.pi/180*vector.new({90,0,-80});
--]]
alpha_L={
{0,0,0,0,0,0,0},
{  0.0011, -0.1707,  0.2745, -0.6262, 0.2900, -0.1883, -0.0142},
{ -0.4935, -0.4341, -0.3611, -0.3445, -0.3146, -0.3083, -0.2258},
{  0.9551,  0.9326,  0.9528,  0.7516, 0.9057,  0.8637,  0.8682},
{ -0.4484, -0.5144, -0.4791, -0.5552, -0.5161, -0.6099, -0.6330},
{  0.0484,  0.2014,  0.0991,  0.2846, 0.1279,  0.1652,  0.0086},
{0,0,0,0,0,0,0},
{  0.0099, -0.1207, -0.0285, -0.6439,  0.0630, -0.1570,  0.0154},
{ -0.2388,  0.0248, -0.6065, -1.4204, -0.9390, -0.3669, -0.5239},
{  0.8967,  0.3473,  2.1253,  3.1100,  1.2767,  0.6664,  0.9784},
{ -0.6579, -0.3721, -1.5188, -1.6895, -0.3377, -0.2995, -0.4545},
{ -0.0099,  0.1207,  0.0285,  0.6439, -0.0630,  0.1570, -0.0154}
}

alpha_R={
{0,0,0,0,0,0,0},
{ -0.0087,  0.1209,  0.0298,  0.6411,-0.0604,  0.1569, -0.0136},
{ -0.2440,  0.0160, -0.6420, -1.3806, -0.9502, -0.3561, -0.5193},
{  0.8991,  0.3576,  2.1616,  3.0429, 1.3304,  0.6534,  0.9772},
{ -0.6550, -0.3736, -1.5196, -1.6623, -0.3802, -0.2973, -0.4579},
{  0.0087, -0.1209, -0.0298, -0.6411, 0.0604, -0.1569,  0.0136},
{0,0,0,0,0,0,0},
{  0.0105,  0.1318, -0.2179,  0.5597, -0.2343,  0.1679,  0.0205},
{ -0.4882, -0.4309,-0.3605, -0.3446, -0.3155, -0.3110, -0.2325},
{  0.9558,  0.9146,  0.9660,  0.7865,  0.8443,  0.9058,  0.8742},
{ -0.4539, -0.5203, -0.4766, -0.5464, -0.5312, -0.5967, -0.6278},
{ -0.0263, -0.2847,  0.0208, -0.4238, -0.0137, -0.2032, -0.0010} 
}

theta_min_L = -0.4484
theta_max_L = -0.6330
theta_min_R = -0.4539
theta_max_R = -0.6278
