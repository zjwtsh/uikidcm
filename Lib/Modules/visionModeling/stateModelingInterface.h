#ifndef STATE_MODELING_OBJECT_H
#define STATE_MODELING_OBJECT_H

extern "C" {

#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"

}

#include <vector>
#include <stdint.h>
#include <filter/bootstrapfilter.h>
#include <model/systemmodel.h>
#include <model/measurementmodel.h>
#include "nonlinearSystemPdf.h"
#include "nonlinearMeasurementPdf.h"

class stateModelingObject
{
public:
	stateModelingObject();
	virtual ~stateModelingObject();
	//bool InitializeEnvironment();
	//bool InitializeBootStrapFilter(MatrixWrapper::ColumnVector initState);
	virtual bool InitializeBootStrapFilter(lua_State *L, int li) = 0;
	virtual bool RunOneStep(void) = 0;
	virtual void clearBootstrap(void);
	virtual bool UpdateStateInfoByLua(lua_State *L, int li) = 0;

protected:
	BFL::NonlinearSystemPdf *psys_pdf;
	BFL::SystemModel<MatrixWrapper::ColumnVector> *psys_model;
	BFL::NonlinearMeasurementPdf *pmeas_pdf;
	BFL::MeasurementModel<preprocessedObservation, MatrixWrapper::ColumnVector> *pmeas_model;
	BFL::MCPdf<MatrixWrapper::ColumnVector> *pprior_discr;
	BFL::BootstrapFilter<MatrixWrapper::ColumnVector, preprocessedObservation> *pfilter;
	preprocessedObservation *pobs;

};

#endif

