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
#include "stateModelingObject.h"

stateModelingObject::stateModelingObject()
{
	psys_pdf = NULL;
	psys_model = NULL;
	pmeas_pdf = NULL;
	pmeas_model = NULL;
	pprior_discr = NULL;
	pfilter = NULL;
	pobs = NULL;
}

virtual void stateModelingObject::clearBootstrap()
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

virtual stateModelingObject::~stateModelingObject()
{
	clearBootstrap();
}

