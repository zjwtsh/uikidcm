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

#include <wrappers/rng/rng.h> // Wrapper around several rng libraries
#include "preprocessedObservation.h"
#include "nonlinearMeasurementPdf.h"

#define MEASMODEL_NUMCONDARGUMENTS_MOBILE 1
#define MEASMODEL_DIMENSION_MOBILE        1

namespace BFL
{
  using namespace MatrixWrapper;
  using namespace std;

  NonlinearMeasurementPdf::NonlinearMeasurementPdf(const Gaussian& measNoise)
    : ConditionalPdf<preprocessedObservation,ColumnVector>(MEASMODEL_DIMENSION_MOBILE,MEASMODEL_NUMCONDARGUMENTS_MOBILE)
  {
    _measNoise = measNoise;
  }

  NonlinearMeasurementPdf::~NonlinearMeasurementPdf(){}

  Probability
  NonlinearMeasurementPdf::ProbabilityGet(const preprocessedObservation& measurement) const
  {
    ColumnVector state = ConditionalArgumentGet(0);
	ColumnVector merror(1); 
	double modelMatchRate;
	double observeMatchRate;
	
	measurement.getTwoMatchRate(state,modelMatchRate,observeMatchRate);
	merror(1) = sqrt((modelMatchRate*modelMatchRate+observeMatchRate*observeMatchRate)/2);

	//cout << "current prob:(" << modelMatchRate <<","<<observeMatchRate<<","<<_measNoise.ProbabilityGet(merror).getValue() <<")"<<endl;

    return _measNoise.ProbabilityGet(merror);
  }

}//namespace BFL

