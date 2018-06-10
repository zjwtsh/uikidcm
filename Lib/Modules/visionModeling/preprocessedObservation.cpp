
//#include "stdafx.h"
//#pragma comment(linker, "/NODEFAULTLIB:libcmt.lib")
#include <iostream>
#include <fstream>
#include <stdint.h>

#include <wrappers/rng/rng.h> // Wrapper around several rng libraries

#include <math.h> //new add 
#include <vector> //new add

#include "preprocessedObservation.h"
#include "block_bitor.h"
#include "ConnectRegions.h"

//#include "lua_color_stats.h"
//#include "lua_goal_posts.h"
//#include "lua_goal_posts_white.h"
//#include "lua_field_lines.h"
//#include "lua_field_spots.h"
//#include "lua_field_occupancy.h"
//#include "lua_robots.h"
#include "lua_accumulate_ball.h"

#define divided_length 5  //the size of devided cell
#define Minlength 25          //minimum length of avalable segments
#define leftboudary -580      //left border in standard field(cm)
#define lowboundary -370      //lower border in standard field(cm)
//#define MAX_SEGMENTS 50       //Maximum number of segments to consider
#define pi 3.1415926          //size of lut_graph

/*
static struct SegmentStats segments[MAX_SEGMENTS];//the number of line segments
//find the available segments to be statistic points:
void available_segments_init()
{
	for (int k = 0; k < MAX_SEGMENTS; k++) {
		int dx = segments[k].x1 - segments[k].x0;
		int dy = segments[k].y1 - segments[k].y0;
		segments[k].length = sqrt(dx*dx + dy*dy);
		if (segments[k].length > Minlength) {
			SegmentStats TEMPOR = { segments[k].x0,segments[k].y0, segments[k].x1,segments[k].y1 };
			available_segments.push_back(TEMPOR);
			valid_segments++;
		}
	}
}
*/





preprocessedObservation::preprocessedObservation(
			double cameraAngleSpead,
			double physicalRadiusOfBall,
			double horizonLimit,
			double noiseRate,
			double radiusRate
			):row(149),col(233),pag(19)
{
	paraCameraAngleSpead = cameraAngleSpead;
	paraPhysicalRadiusOfBall = physicalRadiusOfBall;
	paraHorizonLimit = horizonLimit;
	paraNoiseRate = noiseRate;
	paraRadiusRate = radiusRate;

	lut_graph=NULL;
}

preprocessedObservation::~preprocessedObservation()
{
	clearLutGraph();
}

bool preprocessedObservation::refineParameters(
			double cameraAngleSpead,
			double physicalRadiusOfBall,
			double horizonLimit,
			double noiseRate,
			double radiusRate
			)
{
	paraCameraAngleSpead = cameraAngleSpead;
	paraPhysicalRadiusOfBall = physicalRadiusOfBall;
	paraHorizonLimit = horizonLimit;
	paraNoiseRate = noiseRate;
	paraRadiusRate = radiusRate;
	return true;
}

bool preprocessedObservation::refineObservation(uint8_t *label, int width, int height, double headPitch)
{
	AccumulateParaIn in;

	in.cameraAngleSpead = paraCameraAngleSpead;
	in.physicalRadiusOfBall = paraPhysicalRadiusOfBall;
	in.horizonLimit = paraHorizonLimit;
	in.noiseRate = paraNoiseRate;
	in.radiusRate = paraRadiusRate;
	in.cameraTilt = headPitch; 

	/*
	ballCandidates.clear();
	lua_accumulate_ball(ballCandidates, label, width, height, in);
	*/

	return true;
}

bool preprocessedObservation::getTwoMatchRate(const MatrixWrapper::ColumnVector state, double &modelMatchRate, double &observationMatchRate) const
{
	modelMatchRate = 0.0;
	observationMatchRate = 0.0;
	
	int realweight = 0;//weight of the position
  double tran_x = state(1);
	double tran_y=  state(2);
	double tran_theta = state(3);
	int valid_segments = (int)available_segments.size();

	std::vector <Line_points> linepoint;//useful points to carculate weight
	std::vector <SegmentStats> worked_segments;//segments in world coordinate
	std::vector<SegmentStats> &s = available_segments;	
	SegmentStats tempor;

	//transfer the points from the robotic coordinate to world coordinate
	for (int i = 0; i < valid_segments; i++)
	{
		double x0_temp=s[i].x0;
		double y0_temp=s[i].y0;
		double x1_temp=s[i].x1;
		double y1_temp=s[i].y1;
		tempor.x0 = cos(tran_theta)*x0_temp - sin(tran_theta)*y0_temp + tran_x;
	  tempor.y0 = sin(tran_theta)*x0_temp + cos(tran_theta)*y0_temp + tran_y;
		tempor.x1 = cos(tran_theta)*x1_temp - sin(tran_theta)*y1_temp + tran_x;
		tempor.y1 = sin(tran_theta)*x1_temp + cos(tran_theta)*y1_temp + tran_y;
		worked_segments.push_back(tempor);
	}

	//int p = 0;
  //linepoint.clear();

	//carculate the number of points in lines:
	for (int num = 0; num < valid_segments; num++)
	{
		double kx0 = worked_segments[num].x0;
		double kx1 = worked_segments[num].x1;
		double ky0 = worked_segments[num].y0;
		double ky1 = worked_segments[num].y1;
		//line is vertical
		if (kx0 == kx1)
		{
			int x_temp = round((kx0 - leftboudary) / divided_length) + 1;
			int y_s, y_e;
			if (ky0 <= ky1) 
			{
				y_s = ceil((ky0 - lowboundary) / divided_length) + 1;
				y_e = floor((ky1 - lowboundary) / divided_length) + 1;
			}
			else
			{
				y_s = ceil((ky1 - lowboundary) / divided_length) + 1;
				y_e = floor((ky0 - lowboundary) / divided_length) + 1;
			}
			for (int j = y_s; j <= y_e; j++) 
			{
				Line_points TEMPRO1 = { x_temp ,j ,num ,0 };
				linepoint.push_back(TEMPRO1);
			}
		}
		//line is horizontal
		else if (ky0 == ky1)
		{
			int y_temp = round((ky0 - lowboundary) / divided_length) + 1;
			int x_s, x_e;
			if (kx0 <= kx1) 
			{
				x_s = ceil((kx0 - leftboudary) / divided_length) + 1;
				x_e = floor((kx1 - leftboudary) / divided_length) + 1;
			}
			else 
			{
				x_s = ceil((kx1 - leftboudary) / divided_length) + 1;
				x_e = floor((kx0 - leftboudary) / divided_length) + 1;
			}
			for (int j = x_s; j <= x_e; j++)
			{
				Line_points TEMPRO1 = { j ,y_temp ,num ,90 };
				linepoint.push_back(TEMPRO1);
			}
		}
		else 
		{
			double grad = (ky1 - ky0) / (kx1 - kx0);
			double theta0 = atan(grad) * 180 / pi;
			double b = ky0 - grad*kx0;
			double agrad = fabs(grad);
			//the y direction is the main direction in the increase of step
			if (agrad >= 1) 
			{
				int y_s, y_e;
				if (grad > 0)
				{
					if (ky0 <= ky1)
					{
						y_s = ceil((ky0 - lowboundary) / divided_length) + 1;
						y_e = floor((ky1 - lowboundary) / divided_length) + 1;
					}
					else 
					{
						y_s = ceil((ky1 - lowboundary) / divided_length) + 1;
						y_e = floor((ky0 - lowboundary) / divided_length) + 1;
					}
					double y_rs = divided_length*(y_s - 1) + lowboundary;
					double x_rt = 1 / grad*(y_rs - b);
					for (int j = y_s; j <= y_e; j++) 
					{
						int x_t = round((x_rt - leftboudary) / divided_length) + 1;
						Line_points TEMPRO1 = { x_t ,j ,num ,theta0 };
						linepoint.push_back(TEMPRO1);
						x_rt = x_rt + 1 / grad*divided_length;
					}
				}
				else if (grad < 0)
				{
					if (ky0 <= ky1)
					{
						y_s = floor((ky1 - lowboundary) / divided_length) + 1;
						y_e = ceil((ky0 - lowboundary) / divided_length) + 1;
					}
					else
					{
						y_s = floor((ky0 - lowboundary) / divided_length) + 1;
						y_e = ceil((ky1 - lowboundary) / divided_length) + 1;
					}
					double y_rs = divided_length*(y_s - 1) + lowboundary;
					double x_rt = 1 / grad*(y_rs - b);
					for (int j = y_s; j >= y_e; j--)
					{
						int x_t = round((x_rt - leftboudary) / divided_length) + 1;
						Line_points TEMPRO1 = { x_t ,j ,num ,theta0 };
						linepoint.push_back(TEMPRO1);
						x_rt = x_rt - 1 / grad*divided_length;
					}
				}
			}
			else if (agrad < 1)
			{
				//the x direction is the main direction in the increase of step
				int x_s, x_e;
				if (grad > 0)
				{
					if (kx0 <= kx1)
					{
						x_s = ceil((kx0 - leftboudary) / divided_length) + 1;
						x_e = floor((kx1 - leftboudary) / divided_length) + 1;
					}
					else 
					{
						x_s = ceil((kx1 - leftboudary) / divided_length) + 1;
						x_e = floor((kx0 - leftboudary) / divided_length) + 1;
					}
					double x_rs = divided_length*(x_s - 1) + leftboudary;
					double y_rt = grad*x_rs + b;
					for (int i = x_s; i <= x_e; i++) 
					{
						int y_t = round((y_rt - lowboundary) / divided_length)+ 1;
						Line_points TEMPRO1 = { i ,y_t ,num ,theta0 };
						linepoint.push_back(TEMPRO1);
						y_rt = y_rt + grad*divided_length;
						std::cout << i << " " << y_t << " " << (p-1) << std::endl;
					}
				}
				else if (grad < 0)
				{
					if (kx0 <= kx1)
					{
						x_s = floor((kx1 - leftboudary) / divided_length) + 1;
						x_e = ceil((kx0 - leftboudary) / divided_length) + 1;
					}
					else 
					{
						x_s = floor((kx0 - leftboudary) / divided_length) + 1;
						x_e = ceil((kx1 - leftboudary) / divided_length) + 1;
					}
					double x_rs = divided_length*(x_s - 1) + leftboudary;
					double y_rt = grad*x_rs + b;
					for (int i = x_s; i >= x_e; i--) 
					{
						int y_t = round((y_rt - lowboundary) / divided_length)+ 1;
						Line_points TEMPRO1 = { i ,y_t ,num ,theta0 };
						linepoint.push_back(TEMPRO1);
						y_rt = y_rt - grad*divided_length;
					}
				}
			}
		}
	}

	int point_size=(int)linepoint.size():

	for (int num_0 = 0; num_0 < point_size; num_0++) {
		double theta1= linepoint[num_0].theta / 10;
		int theta_temp = round(theta1) + 9;
		int x_temp = round(linepoint[num_0].x);
		int y_temp = round(linepoint[num_0].y);
		if (((1 <= x_temp) && (x_temp <= col)) && ((1 <= y_temp) && (y_temp <= row)))
			realweight = realweight + lut_graph[y_temp - 1][x_temp - 1][theta_temp];
	}
    realweight = realweight/point_size;
    observationMatchRate=realweight;
	return true;
}

void preprocessedObservation::clearLutGraph(void)
{
	if(lut_graph==NULL)
		return;

	for (int i = 0; i < row; i++)
	{
		for (int j = 0; j < col; j++) {
			delete[] lut_graph[i][j];
		}
		delete[] lut_graph[i];
	}
	delete[] lut_graph;
	lut_graph = NULL;
	return;
}

bool preprocessedObservation::getprolut2map()
{
	std::ifstream fin;
	
	//initial lut_graph
	clearLutGraph();
	
	lut_graph = new uint8_t **[row];
	for (int i = 0; i < row; i++)
	{
		lut_graph[i] = new uint8_t *[col];
		for (int j = 0; j < col; j++) {
			lut_graph[i][j] = new uint8_t[pag];
		}
	}
	
	//load txt uint_8t style
	fin.open("./Data/lut_graph.txt");
	if (!fin.is_open())
	{
		std::cout << "fail to read " << std::endl;
		return false;
	}
	
	int temp = 0;
	for (int c = 0; c < pag; c++)
	{
		for (int a = 0; a < row; a++)
		{
			for (int b = 0; b < col; b++) 
			{
				temp = 0;
				fin >> temp;
				lut_graph[a][b][c] = (uint8_t)temp;
			}
		}
	}
	fin.close();
	std::cout << "lut graph read successfully" << std::endl;

	return true;
}

