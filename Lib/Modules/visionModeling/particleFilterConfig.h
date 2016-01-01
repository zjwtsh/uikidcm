// Copyright (C) 2006 Klaas Gadeyne <first dot last at gmail dot com>
//                    Tinne De Laet <first dot last at mech dot kuleuven dot be>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation; either version 2.1 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
//

#ifndef __PARTICLE_FILTER_CONFIG_
#define __PARTICLE_FILTER_CONFIG_

#include <cmath>

// Sizes
#define STATE_SIZE 3 //state: x,y,theta
#define INPUT_SIZE 3 //input: v*deltat, omega*deltat
#define MEAS_SIZE 1  //USmeasurment: distance_to_wall

#define NUM_SAMPLES 500 // Default Number of Samples
#define RESAMPLE_PERIOD 0 // Default Resample Period
#define RESAMPLE_THRESHOLD (NUM_SAMPLES/3.0) // Threshold for Dynamic Resampling

// Prior:
// Initial estimate of position and orientation
#define PRIOR_MU_X 0.0
#define PRIOR_MU_Y 0.0
#define PRIOR_MU_VEL 0.0

// Initial covariances of position and orientation
#define PRIOR_COV_X pow(30.0,2)
#define PRIOR_COV_Y pow(30.0,2)
#define PRIOR_COV_VEL pow(4.0,2)

// System Noise
#define MU_SYSTEM_NOISE_X 0.0 
#define MU_SYSTEM_NOISE_Y 0.0 
#define MU_SYSTEM_NOISE_VEL 0.0
#define MU_SYSTEM_NOISE_THETA 0.0

#define SIGMA_SYSTEM_NOISE_X pow(3.0 ,2)
#define SIGMA_SYSTEM_NOISE_Y pow(3.0 ,2)
#define SIGMA_SYSTEM_NOISE_VEL pow(1.0, 2)
#define SIGMA_SYSTEM_NOISE_THETA pow(2*M_PI/180,2)

// Measurement noise
#define SIGMA_MEAS_NOISE pow(0.05,2)
#define MU_MEAS_NOISE 0.0

#endif //__MOBILE_ROBOT_CTS
