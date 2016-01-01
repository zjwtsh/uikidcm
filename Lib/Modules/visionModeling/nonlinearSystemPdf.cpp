// $Id: nonlinearanalyticconditionalgaussianmobile.cpp 5823 2005-10-27 13:43:02Z TDeLaet $
// Copyright (C) 2006  Tinne De Laet <first dot last at mech dot kuleuven dot be>
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

#include "nonlinearSystemPdf.h"
#include "particleFilterConfig.h"
#include <wrappers/rng/rng.h> // Wrapper around several rng libraries

#define SYSMODEL_NUMCONDARGUMENTS_MOBILE 2
#define SYSMODEL_DIMENSION_MOBILE        6

namespace BFL
{
  using namespace MatrixWrapper;

  NonlinearSystemPdf::NonlinearSystemPdf(const Gaussian& additiveNoise)
    : ConditionalPdf<ColumnVector,ColumnVector>(SYSMODEL_DIMENSION_MOBILE,SYSMODEL_NUMCONDARGUMENTS_MOBILE)
  {
    _additiveNoise = additiveNoise;
  }

  NonlinearSystemPdf::~NonlinearSystemPdf(){}
  
  bool NonlinearSystemPdf::SampleFrom (Sample<ColumnVector>& one_sample, int method, void * args) const
  {
	  double xk, yk, thetak, velk, extk, rhok, omegak;
	  //double cos_thetak, sin_thetak, cos_omegak, sin_omegak;

    ColumnVector conf = ConditionalArgumentGet(0);
    //ColumnVector vel  = ConditionalArgumentGet(1);
	
	//cout << "before: "<<conf <<endl; 

	xk = conf(1);
	yk = conf(2);
	thetak = conf(3);
	velk = conf(4);
	extk = conf(5);
	rhok = conf(6);
	
	omegak = velk*rhok;
	if (fabs(omegak)<0.1)
	{
		conf(1) += velk*cos(thetak+omegak/2);
		conf(2) += velk*sin(thetak+omegak/2);
	}else
	{
		/*
		cos_thetak = cos(thetak);
		sin_thetak = sin(thetak);
		cos_omegak = cos(omegak);
		sin_omegak = sin(omegak);
		conf(1) += cos_thetak*sin_omegak/rhok-sin_thetak/rhok+sin_thetak*cos_omegak/rhok;
		conf(2) += sin_thetak*sin_omegak/rhok+cos_thetak/rhok-cos_thetak*cos_omegak/rhok;
		*/
		//conf(1) += sin(thetak+omegak)/rhok -sin(thetak)/rhok;
		//conf(2) += -cos(thetak+omegak)/rhok + cos(thetak)/rhok;
		conf(1) += 2*sin(omegak/2)/rhok*cos(thetak+omegak/2);
		conf(2) += 2*sin(omegak/2)/rhok*sin(thetak+omegak/2);
	}

	conf(3) += omegak;
	
    // sample from additive noise
    Sample<ColumnVector> noise;
	ColumnVector noiseVec;
    _additiveNoise.SampleFrom(noise, method, args);
	//cout <<"move : " << conf <<endl; 
	//cout << "noise: "<<noise.ValueGet() <<endl;
	noiseVec = noise.ValueGet();
	if(noiseVec(3)>4.0*sqrt(SIGMA_SYSTEM_NOISE_THETA))
	{
		noiseVec(3) = 2.0*sqrt(SIGMA_SYSTEM_NOISE_THETA)-M_PI;
	}else if(noiseVec(3)>2.0*sqrt(SIGMA_SYSTEM_NOISE_THETA))
	{
		noiseVec(3) = noiseVec(3)-2.0*sqrt(SIGMA_SYSTEM_NOISE_THETA)-M_PI;
	}else if(noiseVec(3)>-2.0*sqrt(SIGMA_SYSTEM_NOISE_THETA))
	{
		noiseVec(3) = noiseVec(3);
	}else if(noiseVec(3)>-4.0*sqrt(SIGMA_SYSTEM_NOISE_THETA))
	{
		noiseVec(3) = noiseVec(3) + 2.0*sqrt(SIGMA_SYSTEM_NOISE_THETA) + M_PI;
	}else
	{
		noiseVec(3) = -2.0*sqrt(SIGMA_SYSTEM_NOISE_THETA)+M_PI;
	}

	conf = conf + noiseVec;

	if (conf(3) < -M_PI)
	{
		//cout << "previous:" <<conf(3)*180/M_PI<<endl;
		conf(3) = conf(3) + int(conf(3)/2/M_PI)*2*M_PI + 2*M_PI;
		//cout << "posterious:" <<conf(3)*180/M_PI<<endl;
	}else if (conf(3) > M_PI)
	{
		//cout << "previous:" <<conf(3)*180/M_PI<<endl;
		conf(3) = conf(3) - int(conf(3)/2/M_PI)*2*M_PI - 2*M_PI;
		//cout << "posterious:" <<conf(3)*180/M_PI<<endl;
	}

	if(conf(4)<-1)
		conf(4) = -1;
	else if(conf(4)>20)
		conf(4)= 20;

	if(conf(5)<0)
		conf(5) = 0;
	else if(conf(5)>30)
		conf(5) = 30;


	if(conf(6)>0.06)
		conf(6) = 0.06;
	else if(conf(6)<-0.06)
		conf(6) = -0.06;

	//cout << "after: "<<conf <<endl;

    // store results in one_sample
    one_sample.ValueSet(conf);

    return true;
  }

}//namespace BFL

