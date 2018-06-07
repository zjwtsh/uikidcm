
//#include "stdafx.h"
//#pragma comment(linker, "/NODEFAULTLIB:libcmt.lib")
#include <iostream>
#include <fstream>
#include <stdint.h>

#include <wrappers/rng/rng.h> // Wrapper around several rng libraries

#include "preprocessedObservation.h"
#include "block_bitor.h"
#include "ConnectRegions.h"

//#include "lua_color_stats.h"
//#include "lua_goal_posts.h"
//#include "lua_goal_posts_white.h"
//#include "lua_field_lines.h"
//#include "lua_field_spots.h"
//#include "lua_field_occupancy.h"
//#include "lua_robots.h"
#include "lua_accumulate_ball.h"

preprocessedObservation::preprocessedObservation(
			double cameraAngleSpead,
			double physicalRadiusOfBall,
			double horizonLimit,
			double noiseRate,
			double radiusRate
			)
{
	paraCameraAngleSpead = cameraAngleSpead;
	paraPhysicalRadiusOfBall = physicalRadiusOfBall;
	paraHorizonLimit = horizonLimit;
	paraNoiseRate = noiseRate;
	paraRadiusRate = radiusRate;
}

bool preprocessedObservation::refineParameters(
			double cameraAngleSpead,
			double physicalRadiusOfBall,
			double horizonLimit,
			double noiseRate,
			double radiusRate
			)
{
	paraCameraAngleSpead = cameraAngleSpead;
	paraPhysicalRadiusOfBall = physicalRadiusOfBall;
	paraHorizonLimit = horizonLimit;
	paraNoiseRate = noiseRate;
	paraRadiusRate = radiusRate;
	return true;
}

bool preprocessedObservation::refineObservation(uint8_t *label, int width, int height, double headPitch)
{
	AccumulateParaIn in;

	in.cameraAngleSpead = paraCameraAngleSpead;
	in.physicalRadiusOfBall = paraPhysicalRadiusOfBall;
	in.horizonLimit = paraHorizonLimit;
	in.noiseRate = paraNoiseRate;
	in.radiusRate = paraRadiusRate;
	in.cameraTilt = headPitch; 

	ballCandidates.clear();
	lua_accumulate_ball(ballCandidates, label, width, height, in);

	return true;
}

bool preprocessedObservation::getTwoMatchRate(const MatrixWrapper::ColumnVector state, double &modelMatchRate, double &observationMatchRate) const
{
	modelMatchRate = 0.0;
	observationMatchRate = 0.0;
	return true;
}


