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

#include "preprocessedObservation.h"
#include "ballModelingObject.h"
#include "particleFilterConfig.h"

bool ballModelingObject::ExtractLineInfoByLua(lua_State *L, int li)
{
	std::string str;
	size_t sz = 0;
	int detect = 0;
	double nlines = 0;
	//unsigned char indices[][5] = {"v"}
	//std::cout << "extraction function in ballModelingObject is called" << std::endl;

	if(pobs==NULL)
		return false;

	if(!lua_istable(L,li))
		luaL_error(L,"invalid observations for vision modeling");

	lua_getfield(L,li,"name");
	str = luaL_checklstring(L,-1,&sz);
	lua_pop(L,1);

	lua_getfield(L,li,"detect");
	detect = (int)(luaL_checknumber(L,-1)+0.5);
	lua_pop(L,1);

	lua_getfield(L,li,"nLines");
	nlines = luaL_checknumber(L,-1);
	lua_pop(L,1);

	std::cout << str << " " <<detect << " "<< nlines<<std::endl;
	pobs->available_segments.clear();

	if(detect==0)
	{
		return true;
	}

	int ts = 0;
	lua_getfield(L,li,"v");
	if(!lua_istable(L,-1))
		luaL_error(L,"invalid observations for vision modeling");
	ts = lua_objlen(L,-1);

	for(int i = 0; i<ts; i++)
	{
		int ts2=0;
		struct SegmentStats lf;
		double *plf = (double *)&lf;

		lua_rawgeti(L, -1,i+1);
		if(!lua_istable(L,-1))
			luaL_error(L,"invalid observations for vision modeling");
		ts2 = lua_objlen(L,-1);

		for(int j= 0;j<ts2;j++)
		{
			double value = 0;
			lua_rawgeti(L,-1,j+1);
			value = luaL_checknumber(L, -1);
			plf[j] = value;
			//std::cout << lf.v[j] << ", ";
			lua_pop(L,1);
		}
		pobs->available_segments.push_back(lf);
		//std::cout << std::endl;
		lua_pop(L,1);
	}
	lua_pop(L,1);

	for(int i=0;i<pobs->available_segments.size();i++)
	{
		std::cout << i << ":";
		std::cout<<pobs->available_segments[i].x0 << " ";
		std::cout<<pobs->available_segments[i].y0 << " ";
		std::cout<<pobs->available_segments[i].x1 << " ";
		std::cout<<pobs->available_segments[i].y1 << " ";
		std::cout<<std::endl;
	}

	return true;
}

ballModelingObject::ballModelingObject()
{
	psys_pdf = NULL;
	psys_model = NULL;
	pmeas_pdf = NULL;
	pmeas_model = NULL;
	pprior_discr = NULL;
	pfilter = NULL;
	pobs = NULL;
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
	if(pobs != NULL)
		delete pobs;

	psys_pdf = NULL;
	psys_model = NULL;
	pmeas_pdf = NULL;
	pmeas_model = NULL;
	pprior_discr = NULL;
	pfilter = NULL;
	pobs = NULL;

	return;
}

ballModelingObject::~ballModelingObject()
{
	clearBootstrap();
}

bool ballModelingObject::InitializeBootStrapFilter(MatrixWrapper::ColumnVector initState)
{
	pobs = new preprocessedObservation(50*M_PI/180, 90, 5*M_PI/180, 0.3, 1.2);
	pobs->getprolut2map();
	//copy code from mouseBehaviorGenerator
	MatrixWrapper::ColumnVector sys_noise_Mu(STATE_SIZE);
	sys_noise_Mu(1) = MU_SYSTEM_NOISE_X;
	sys_noise_Mu(2) = MU_SYSTEM_NOISE_Y;
	sys_noise_Mu(3) = MU_SYSTEM_NOISE_VEL;
	std::cout<< "sys_noise_Mu = " <<sys_noise_Mu <<std::endl ;

	MatrixWrapper::SymmetricMatrix sys_noise_Cov(STATE_SIZE);
	sys_noise_Cov = 0.0;
	sys_noise_Cov(1,1) = SIGMA_SYSTEM_NOISE_X;
	sys_noise_Cov(2,2) = SIGMA_SYSTEM_NOISE_Y;
	sys_noise_Cov(3,3) = SIGMA_SYSTEM_NOISE_VEL;
	std::cout<< " sys_noise_Cov = " <<sys_noise_Cov <<std::endl ;

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
	std::cout<< "prior_Mu = " <<prior_Mu <<std::endl ;

	MatrixWrapper::SymmetricMatrix prior_Cov(STATE_SIZE);
	prior_Cov = 0.0; 
	prior_Cov(1,1) = PRIOR_COV_X;
	prior_Cov(2,2) = PRIOR_COV_Y;
	prior_Cov(3,3) = PRIOR_COV_VEL;

	BFL::Gaussian prior_cont(initState,prior_Cov);

	// Discrete prior for Particle filter (using the continuous Gaussian prior)
	std::vector<BFL::Sample<MatrixWrapper::ColumnVector> > prior_samples(NUM_SAMPLES);
	pprior_discr = new BFL::MCPdf<MatrixWrapper::ColumnVector>(NUM_SAMPLES,STATE_SIZE);
	prior_cont.SampleFrom(prior_samples,NUM_SAMPLES,CHOLESKY,NULL);
	pprior_discr->ListOfSamplesSet(prior_samples);

	std::vector< BFL::Sample<MatrixWrapper::ColumnVector> >::const_iterator iter; 
	for (iter = prior_samples.begin(); iter != prior_samples.end(); iter++)
	{
		//std::cout<< iter->ValueGet() << std::endl;
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

bool ballModelingObject::UpdateBallParameters()
{
	//images are processed to obtain observations
	/*
	cameraAngleSpead = 50*M_PI/180;
	physicalRadiusOfBall = 90;
	horizonLimit = 5*M_PI/180;
	noiseRate = 0.3;
	radiusRate = 1.2;
	*/

	pobs->refineParameters(50*M_PI/180.0, 90, 5*M_PI/180.0, 0.3, 1.2);
	return true;
}

bool ballModelingObject::RunOneStep()
{
	//the next step is update of particle filter
	MatrixWrapper::ColumnVector input(2);
	input(1) = 0.0;
	input(2) = 0.0;

	pfilter->Update(psys_model, input, pmeas_model, *pobs);

	return true;
}

