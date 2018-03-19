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
	return true;
}

bool ballModelingObject::RunOneStep()
{
	return true;
}

