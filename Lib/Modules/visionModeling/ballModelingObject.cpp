#ifdef __cplusplus
extern "C" {
#endif

#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"

#ifdef __cplusplus
}
#endif

#include <stdint.h>
#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <vector>

#include <filter/bootstrapfilter.h>
#include <model/systemmodel.h>
#include <model/measurementmodel.h>
#include "nonlinearSystemPdf.h"
#include "nonlinearMeasurementPdf.h"

#include "preprocessedObservation.h"
#include "ballModelingObject.h"

ballModelingObject::ballModelingObject()
{
	psys_pdf = NULL;
	psys_model = NULL;
	pmeas_pdf = NULL;
	pmeas_model = NULL;
	pprior_discr = NULL;
	pfilter = NULL;
}

void ballModelingObject::clearBootstrap()
{
	if(pfilter !=NULL)
		delete pfilter;
	if(pprior_discr !=NULL)
		delete pprior_discr;
	if(pmeas_model != NULL)
		delete pmeas_model;
	if(pmeas_pdf != NULL)
		delete pmeas_pdf;
	if(psys_model != NULL)
		delete psys_model;
	if(psys_pdf != NULL)
		delete psys_pdf;
	return;
}

ballModelingObject::~ballModelingObject()
{
	clearBootstrap();
}

bool ballModelingObject::InitializeBootStrapFilter(MatrixWrapper::ColumnVector initState)
{
	//copy code from mouseBehaviorGenerator
	MatrixWrapper::ColumnVector sys_noise_Mu(STATE_SIZE);
	sys_noise_Mu(1) = MU_SYSTEM_NOISE_X;
	sys_noise_Mu(2) = MU_SYSTEM_NOISE_Y;
	sys_noise_Mu(3) = MU_SYSTEM_NOISE_VEL;
	std::cout<< "sys_noise_Mu = " <<sys_noise_Mu <<endl ;

	MatrixWrapper::SymmetricMatrix sys_noise_Cov(STATE_SIZE);
	sys_noise_Cov = 0.0;
	sys_noise_Cov(1,1) = SIGMA_SYSTEM_NOISE_X;
	sys_noise_Cov(2,2) = SIGMA_SYSTEM_NOISE_Y;
	sys_noise_Cov(3,3) = SIGMA_SYSTEM_NOISE_VEL;
	std::cout<< " sys_noise_Cov = " <<sys_noise_Cov <<endl ;

	BFL::Gaussian system_Uncertainty(sys_noise_Mu, sys_noise_Cov);
	psys_pdf = new BFL::NonlinearSystemPdf(system_Uncertainty);
	psys_model = new BFL::SystemModel<MatrixWrapper::ColumnVector> (psys_pdf);

	MatrixWrapper::ColumnVector meas_noise_Mu(MEAS_SIZE);
	meas_noise_Mu(1) = MU_MEAS_NOISE;
	MatrixWrapper::SymmetricMatrix meas_noise_Cov(MEAS_SIZE);
	meas_noise_Cov(1,1) = SIGMA_MEAS_NOISE;
	BFL::Gaussian measurement_Uncertainty(meas_noise_Mu, meas_noise_Cov);

	pmeas_pdf = new BFL::NonlinearMeasurementPdf(measurement_Uncertainty);
	pmeas_model = new BFL::MeasurementModel<preprocessedObservation,MatrixWrapper::ColumnVector>(pmeas_pdf);

	MatrixWrapper::ColumnVector prior_Mu(STATE_SIZE);
	prior_Mu = initState ;
	std::cout<< "prior_Mu = " <<prior_Mu <<endl ;

	MatrixWrapper::SymmetricMatrix prior_Cov(STATE_SIZE);
	prior_Cov = 0.0; 
	prior_Cov(1,1) = PRIOR_COV_X;
	prior_Cov(2,2) = PRIOR_COV_Y;
	prior_Cov(3,3) = PRIOR_COV_VEL;

	BFL::Gaussian prior_cont(initState,prior_Cov);

	// Discrete prior for Particle filter (using the continuous Gaussian prior)
	vector<BFL::Sample<MatrixWrapper::ColumnVector> > prior_samples(NUM_SAMPLES);
	pprior_discr = new BFL::MCPdf<MatrixWrapper::ColumnVector>(NUM_SAMPLES,STATE_SIZE);
	prior_cont.SampleFrom(prior_samples,NUM_SAMPLES,CHOLESKY,NULL);
	pprior_discr->ListOfSamplesSet(prior_samples);

	vector<BFL::Sample<MatrixWrapper::ColumnVector>>::const_iterator iter; 
	for (iter = prior_samples.begin(); iter != prior_samples.end(); iter++)
	{
		std::cout<< iter->ValueGet() << std::endl;
		//drawParticle(img,iter->ValueGet());
		//cv::imshow("debug2",img);
		//cv::waitKey(2000);
	}

	/******************************
	* Construction of the Filter *
	******************************/
	pfilter = new BFL::BootstrapFilter<MatrixWrapper::ColumnVector,preprocessedObservation>(pprior_discr, 0, NUM_SAMPLES/4.0);

	return true;
}

bool ballModelingObject::RunOneStep()
{
	return true;
}

