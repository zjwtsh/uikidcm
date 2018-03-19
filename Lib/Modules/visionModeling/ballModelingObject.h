#ifndef lua_accumulate_ball_h_DEFINED
#define lua_accumulate_ball_h_DEFINED

#include <stdint.h>

class ballModelingObject
{
public:
	ballModelingObject();
	~ballModelingObject();
	//bool InitializeEnvironment();
	bool InitializeBootStrapFilter(MatrixWrapper::ColumnVector initState);
	bool RunOneStep();

protected:
	void clearBootstrap(void);
	bool refineObservation(); //this is the function to obtain the ball candidates;

protected:
	BFL::NonlinearSystemPdf *psys_pdf;
	BFL::SystemModel<MatrixWrapper::ColumnVector> *psys_model;
	BFL::NonlinearMeasurementPdf *pmeas_pdf;
	BFL::MeasurementModel<preprocessObservation, MatrixWrapper::ColumnVector> *pmeas_model;
	BFL::MCPdf<MatrixWrapper::ColumnVector> *pprior_discr;
	BFL::BootstrapFilter<MatrixWrapper::ColumnVector, preprocessedObservation> *pfilter;

};

#endif
