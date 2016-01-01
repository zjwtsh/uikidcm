#ifndef PREPROCESSED_OBSERVATION_H
#define PREPROCESSED_OBSERVATION_H

#include <iostream>
#include <fstream>
#include <vector>

#include <filter/bootstrapfilter.h>

#include <model/systemmodel.h>
#include <model/measurementmodel.h>

#include "particleFilterConfig.h"
#include "lua_accumulate_ball.h"

/*
struct LineInfo
{
	double v[4];
};
*/

struct SegmentStats {
	double x0;
	double y0;//start point
	double x1;
	double y1;//end point
	//double length;
};

struct Line_points{
	int x;
	int y;
	int index;
	double theta;
};

class preprocessedObservation
{
public:
	preprocessedObservation(
			double cameraAngleSpead = 50*M_PI/180,
			double physicalRadiusOfBall = 90,
			double horizonLimit = 5*M_PI/180,
			double noiseRate = 0.3,
			double radiusRate = 1.2
			);

	bool refineObservation(uint8_t *label, int width, int height, double headPitch);

	bool refineParameters(
			double cameraAngleSpead = 50*M_PI/180,
			double physicalRadiusOfBall = 90,
			double horizonLimit = 5*M_PI/180,
			double noiseRate = 0.3,
			double radiusRate = 1.2
			);
	bool getTwoMatchRate(const MatrixWrapper::ColumnVector state, double &modelMatchRate, double &observationMatchRate) const;
	
	bool getprolut2map();
	~preprocessedObservation();

protected:
	void clearLutGraph(void);
	//void coor_trans(MatrixWrapper::ColumnVector state);
	//int plot_lines(void);
	//int coor_tans_and_plot_lines(MatrixWrapper::ColumnVector state) const;

protected:
	uint8_t ***lut_graph;
	int row;	//row of lut_graph
	int col;	//column of lut_graph
	int pag;	//page of lut_graph

public:
	//std::vector<struct LineInfo> lineInfo;
	std::vector <SegmentStats> available_segments;	//the real effect number of line segments

protected:
	double paraCameraAngleSpead;
	double paraPhysicalRadiusOfBall;
	double paraHorizonLimit;
	double paraNoiseRate;
	double paraRadiusRate;
	
	//std::vector<Candidate> ballCandidates;
	/*
	void drawCurrentModel(MatrixWrapper::ColumnVector &state, cv::Mat &model);
	void drawModelObservation(cv::Mat &img, MatrixWrapper::ColumnVector &state);
	void drawModelObservation2(cv::Mat &img, MatrixWrapper::ColumnVector &state);

protected:
	void drawCurrentModel(const MatrixWrapper::ColumnVector &state, cv::Mat &model, cv::Rect &aabb) const; 

protected:
	cv::Mat _filteredImage;
	cv::Mat _model;
	std::vector< std::vector<cv::Point> > _contours; 
	long _contourArea;
	*/
};

#endif


