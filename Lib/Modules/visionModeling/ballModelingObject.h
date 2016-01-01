#ifndef BALL_MODELING_OBJECT_H
#define BALL_MODELING_OBJECT_H

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
#include "stateModelingObject.h"

class ballModelingObject: public stateModelingObject
{
public:
	ballModelingObject();
	virtual ~ballModelingObject();
	virtual bool InitializeBootStrapFilter(void);
	virtual bool RunOneStep(void);
	virtual void clearBootstrap(void);
	virtual bool UpdateStateInfoByLua(lua_State *L, int li);

protected:
	bool refineObservation(); //this is the function to obtain the ball candidates;
	bool UpdateBallParameters(void);

};

#endif


