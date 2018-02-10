// $Id: test_nonlinear_particle.cpp 5925 2006-03-14 21:23:49Z tdelaet $
// Copyright (C) 2006 Tinne De Laet <first dot last at mech dot kuleuven dot be>
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

/* Demonstration program for the Bayesian Filtering Library.
   Mobile robot localization with respect to wall with different possibilities for filter
*/


#include <filter/bootstrapfilter.h>

#include <model/systemmodel.h>
#include <model/measurementmodel.h>

#include "nonlinearSystemPdf.h"
#include "nonlinearMeasurementPdf.h"

#include <iostream>
#include <fstream>

using namespace MatrixWrapper;
using namespace BFL;
using namespace std;

// Sizes
#define STATE_SIZE	4 //state: x, y, vx, vy, 
#define INPUT_SIZE	3 //input: robot_step in x, y and theta components
#define MEAS_SIZE	1 //measurement: special measurements with multi ball observation candidates 

#define DEFAULT_NUM_SAMPLES	1000			// Default Number of Samples
#define RESAMPLE_PERIOD		0				// Default Resample Period
#define RESAMPLE_THRESHOLD (NUM_SAMPLES/4.0) // Default Threshold for Dynamic Resampling

#define MU_SYSTEM_NOISE_X 0.0 
#define MU_SYSTEM_NOISE_Y 0.0 
#define MU_SYSTEM_NOISE_VX 0.0
#define MU_SYSTEM_NOISE_VY 0.0
#define SIGMA_SYSTEM_NOISE_X pow(0.01,2)
#define SIGMA_SYSTEM_NOISE_Y pow(0.01,2)
#define SIGMA_SYSTEM_NOISE_VX pow(2*M_PI/180,2)
#define SIGMA_SYSTEM_NOISE_VY pow(2*M_PI/180,2)

int InitializeBallModeling(void)
{
  /****************************
   * NonLinear system model      *
   ***************************/

  // create gaussian
  ColumnVector sys_noise_Mu(STATE_SIZE);
  sys_noise_Mu(1) = MU_SYSTEM_NOISE_X;
  sys_noise_Mu(2) = MU_SYSTEM_NOISE_Y;
  sys_noise_Mu(3) = MU_SYSTEM_NOISE_VX;
  sys_noise_Mu(4) = MU_SYSTEM_NOISE_VY;

  SymmetricMatrix sys_noise_Cov(STATE_SIZE);
  sys_noise_Cov = 0.0;
  sys_noise_Cov(1,1) = SIGMA_SYSTEM_NOISE_X;
  sys_noise_Cov(1,2) = 0.0;
  sys_noise_Cov(1,3) = 0.0;
  sys_noise_Cov(1,4) = 0.0;
  sys_noise_Cov(2,1) = 0.0;
  sys_noise_Cov(2,2) = SIGMA_SYSTEM_NOISE_Y;
  sys_noise_Cov(2,3) = 0.0;
  sys_noise_Cov(2,4) = 0.0;
  sys_noise_Cov(3,1) = 0.0;
  sys_noise_Cov(3,2) = 0.0;
  sys_noise_Cov(3,3) = SIGMA_SYSTEM_NOISE_VX;
  sys_noise_Cov(3,4) = 0.0;
  sys_noise_Cov(4,1) = 0.0;
  sys_noise_Cov(4,2) = 0.0;
  sys_noise_Cov(4,3) = 0.0;
  sys_noise_Cov(4,4) = SIGMA_SYSTEM_NOISE_VY;

  Gaussian system_Uncertainty(sys_noise_Mu, sys_noise_Cov);
  NonlinearSystemPdf sys_pdf(system_Uncertainty);
  SystemModel<ColumnVector> sys_model(&sys_pdf);

  // Construct the measurement noise (a scalar in this case)
  ColumnVector meas_noise_Mu(MEAS_SIZE);
  meas_noise_Mu(1) = MU_MEAS_NOISE;
  SymmetricMatrix meas_noise_Cov(MEAS_SIZE);
  meas_noise_Cov(1,1) = SIGMA_MEAS_NOISE;
  Gaussian measurement_Uncertainty(meas_noise_Mu, meas_noise_Cov);

  // create the measurement model
  LinearAnalyticConditionalGaussian meas_pdf(H, measurement_Uncertainty);
  LinearAnalyticMeasurementModelGaussianUncertainty meas_model(&meas_pdf);

  /****************************
   * Linear prior DENSITY     *
   ***************************/
  // Continuous Gaussian prior (for Kalman filters)
  ColumnVector prior_Mu(STATE_SIZE);
  prior_Mu(1) = PRIOR_MU_X;
  prior_Mu(2) = PRIOR_MU_Y;
  prior_Mu(3) = PRIOR_MU_THETA;
  SymmetricMatrix prior_Cov(STATE_SIZE);
  prior_Cov(1,1) = PRIOR_COV_X;
  prior_Cov(1,2) = 0.0;
  prior_Cov(1,3) = 0.0;
  prior_Cov(2,1) = 0.0;
  prior_Cov(2,2) = PRIOR_COV_Y;
  prior_Cov(2,3) = 0.0;
  prior_Cov(3,1) = 0.0;
  prior_Cov(3,2) = 0.0;
  prior_Cov(3,3) = PRIOR_COV_THETA;
  Gaussian prior_cont(prior_Mu,prior_Cov);

  // Discrete prior for Particle filter (using the continuous Gaussian prior)
  vector<Sample<ColumnVector> > prior_samples(NUM_SAMPLES);
  MCPdf<ColumnVector> prior_discr(NUM_SAMPLES,STATE_SIZE);
  
  prior_cont.SampleFrom(prior_samples,NUM_SAMPLES,CHOLESKY,NULL);
  prior_discr.ListOfSamplesSet(prior_samples);

  /******************************
   * Construction of the Filter *
   ******************************/
  BootstrapFilter<ColumnVector,ColumnVector> filter(&prior_discr, 0, NUM_SAMPLES/4.0);

	return 0;
}

int UpdateBallModeling(void)
{
	/*
	 * MobileRobot mobile_robot;
	 * ColumnVector input(2);
	 * input(1) = 0.1;
	 * input(2) = 0.0;
	 * mobile_robot.Move(input);
	 */

      // DO ONE MEASUREMENT
      ColumnVector measurement = mobile_robot.Measure();

      // UPDATE FILTER
      filter.Update(&sys_model,input,&meas_model,measurement);

  Pdf<ColumnVector> * posterior = filter.PostGet();
  cout << "After " << time_step+1 << " timesteps " << endl;
  cout << " Posterior Mean = " << endl << posterior->ExpectedValueGet() << endl
       << " Covariance = " << endl << posterior->CovarianceGet() << "" << endl;


  cout << "======================================================" << endl
       << "End of the Bootstrap filter for mobile robot localisation" << endl
       << "======================================================"
       << endl;



	return 0;
}

int main(int argc, char** argv)
{
  cerr << "==================================================" << endl
       << "Test of bootstrap filter" << endl
       << "Mobile robot localisation example" << endl
       << "==================================================" << endl;


  /****************************
   * NonLinear system model      *
   ***************************/

  // create gaussian
  ColumnVector sys_noise_Mu(STATE_SIZE);
  sys_noise_Mu(1) = MU_SYSTEM_NOISE_X;
  sys_noise_Mu(2) = MU_SYSTEM_NOISE_Y;
  sys_noise_Mu(3) = MU_SYSTEM_NOISE_THETA;

  SymmetricMatrix sys_noise_Cov(STATE_SIZE);
  sys_noise_Cov = 0.0;
  sys_noise_Cov(1,1) = SIGMA_SYSTEM_NOISE_X;
  sys_noise_Cov(1,2) = 0.0;
  sys_noise_Cov(1,3) = 0.0;
  sys_noise_Cov(2,1) = 0.0;
  sys_noise_Cov(2,2) = SIGMA_SYSTEM_NOISE_Y;
  sys_noise_Cov(2,3) = 0.0;
  sys_noise_Cov(3,1) = 0.0;
  sys_noise_Cov(3,2) = 0.0;
  sys_noise_Cov(3,3) = SIGMA_SYSTEM_NOISE_THETA;

  Gaussian system_Uncertainty(sys_noise_Mu, sys_noise_Cov);

  // create the nonlinear system model
  NonlinearSystemPdf sys_pdf(system_Uncertainty);
  SystemModel<ColumnVector> sys_model(&sys_pdf);


  /*********************************
   * NonLinear Measurement model   *
   ********************************/

  // create matrix H for linear measurement model
  double wall_ct = 2/(sqrt(pow(RICO_WALL,2.0) + 1));
  Matrix H(MEAS_SIZE,STATE_SIZE);
  H = 0.0;
  H(1,1) = wall_ct * RICO_WALL;
  H(1,2) = 0 - wall_ct;
  H(1,3) = 0.0;
  // Construct the measurement noise (a scalar in this case)
  ColumnVector meas_noise_Mu(MEAS_SIZE);
  meas_noise_Mu(1) = MU_MEAS_NOISE;
  SymmetricMatrix meas_noise_Cov(MEAS_SIZE);
  meas_noise_Cov(1,1) = SIGMA_MEAS_NOISE;
  Gaussian measurement_Uncertainty(meas_noise_Mu, meas_noise_Cov);

  // create the measurement model
  LinearAnalyticConditionalGaussian meas_pdf(H, measurement_Uncertainty);
  LinearAnalyticMeasurementModelGaussianUncertainty meas_model(&meas_pdf);

  /****************************
   * Linear prior DENSITY     *
   ***************************/
  // Continuous Gaussian prior (for Kalman filters)
  ColumnVector prior_Mu(STATE_SIZE);
  prior_Mu(1) = PRIOR_MU_X;
  prior_Mu(2) = PRIOR_MU_Y;
  prior_Mu(3) = PRIOR_MU_THETA;
  SymmetricMatrix prior_Cov(STATE_SIZE);
  prior_Cov(1,1) = PRIOR_COV_X;
  prior_Cov(1,2) = 0.0;
  prior_Cov(1,3) = 0.0;
  prior_Cov(2,1) = 0.0;
  prior_Cov(2,2) = PRIOR_COV_Y;
  prior_Cov(2,3) = 0.0;
  prior_Cov(3,1) = 0.0;
  prior_Cov(3,2) = 0.0;
  prior_Cov(3,3) = PRIOR_COV_THETA;
  Gaussian prior_cont(prior_Mu,prior_Cov);

  // Discrete prior for Particle filter (using the continuous Gaussian prior)
  vector<Sample<ColumnVector> > prior_samples(NUM_SAMPLES);
  MCPdf<ColumnVector> prior_discr(NUM_SAMPLES,STATE_SIZE);
  prior_cont.SampleFrom(prior_samples,NUM_SAMPLES,CHOLESKY,NULL);
  prior_discr.ListOfSamplesSet(prior_samples);

  /******************************
   * Construction of the Filter *
   ******************************/
  BootstrapFilter<ColumnVector,ColumnVector> filter(&prior_discr, 0, NUM_SAMPLES/4.0);

  /***************************
   * initialise MOBILE ROBOT *
   **************************/
  // Model of mobile robot in world with one wall
  // The model is used to simultate the distance measurements.
  MobileRobot mobile_robot;
  ColumnVector input(2);
  input(1) = 0.1;
  input(2) = 0.0;




  /*******************
   * ESTIMATION LOOP *
   *******************/
  cout << "MAIN: Starting estimation" << endl;
  unsigned int time_step;
  for (time_step = 0; time_step < NUM_TIME_STEPS-1; time_step++)
    {
      // DO ONE STEP WITH MOBILE ROBOT
      mobile_robot.Move(input);

      // DO ONE MEASUREMENT
      ColumnVector measurement = mobile_robot.Measure();

      // UPDATE FILTER
      filter.Update(&sys_model,input,&meas_model,measurement);

    } // estimation loop



  Pdf<ColumnVector> * posterior = filter.PostGet();
  cout << "After " << time_step+1 << " timesteps " << endl;
  cout << " Posterior Mean = " << endl << posterior->ExpectedValueGet() << endl
       << " Covariance = " << endl << posterior->CovarianceGet() << "" << endl;


  cout << "======================================================" << endl
       << "End of the Bootstrap filter for mobile robot localisation" << endl
       << "======================================================"
       << endl;


  return 0;
}
