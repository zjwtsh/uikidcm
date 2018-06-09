
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

using namespace std;

#define divided_length 5  //the size of devided cell
#define Minlength 25          //minimum length of avalable segments
#define leftboudary -580      //left border in standard field(cm)
#define lowboundary -370      //lower border in standard field(cm)
#define MAX_SEGMENTS 50       //Maximum number of segments to consider
#define pi 3.1415926          //size of lut_graph

const int row = 149;//row of lut_graph
const int col = 233;//column of lut_graph
const int pag = 19; //page of lut_graph

static int valid_segments = 0;
struct SegmentStats {
	double x0;
	double y0;//start point
	double x1;
	double y1;//end point
	double length;
};
struct State {
	double x;//x-coordinate in world 
	double y;//y_coordinete in world
	double theta;//theta in world coordinate
};
struct Line_points{
	int x;
	int y;
	int index;
	double theta;
};
static struct SegmentStats segments[MAX_SEGMENTS];//the number of line segments
static vector <SegmentStats> available_segments;//the real effect number of line segments
static vector <Line_points> linepoint;//the points number&location of the effect lines

//find the available segments to be statistic points:
void preprocessedObservation::available_segments_init()
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

//transfer the points from the robotic coordinate to world coordinate:
void preprocessedObservation::coor_trans(SegmentStats s[], State STATE)
{
	double tran_x = STATE.x;
	double tran_y=  STATE.y;
	double tran_theta = STATE.theta*pi/180;
	for (int i = 0; i < MAX_SEGMENTS; i++)
	{
		double x0_temp=s[i].x0;
		double y0_temp=s[i].y0 ;
		double x1_temp=s[i].x1 ;
		double y1_temp=s[i].y1 ;
		s[i].x0 = cos(tran_theta)*x0_temp - sin(tran_theta)*y0_temp + tran_x;
	  s[i].y0 = sin(tran_theta)*x0_temp + cos(tran_theta)*y0_temp + tran_y;
		s[i].x1 = cos(tran_theta)*x1_temp - sin(tran_theta)*y1_temp + tran_x;
		s[i].y1 = sin(tran_theta)*x1_temp + cos(tran_theta)*y1_temp + tran_y;
	}
}

//carculate the number of points in lines:
int preprocessedObservation::plot_lines() 
{
	int p = 0;
	for (int num = 0; num < valid_segments; num++)
	{
		double kx0 = available_segments[num].x0;
		double kx1 = available_segments[num].x1;
		double ky0 = available_segments[num].y0;
		double ky1 = available_segments[num].y1;
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
				p++;
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
				p++;
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
						p++;
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
						p++;
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
						p++;
						y_rt = y_rt + grad*divided_length;
						//cout << i << " " << y_t << " " << (p-1) << endl;
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
						p++;
						y_rt = y_rt - grad*divided_length;
					}
				}
			}
		}
	}
	return p;
}


static uint8_t ***lut_graph;

preprocessedObservation::preprocessedObservation(
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
	return true;
}

bool preprocessedObservation::getTwoMatchRate(const MatrixWrapper::ColumnVector state, double &modelMatchRate, double &observationMatchRate) const
{
	getprolut2map();
	State state1;
	coor_trans(segments, state1);
	available_segments_init();
	int size = plot_lines();
	int realweight = 0;
	for (int num_0 = 0; num_0 < size; num_0++) {
		double theta1= linepoint[num_0].theta / 10;
		int theta_temp = round(theta1) + 9;
		int x_temp = round(linepoint[num_0].x);
		int y_temp = round(linepoint[num_0].y);
		if (((1 <= x_temp) && (x_temp <= col)) && ((1 <= y_temp) && (y_temp <= row)))
			realweight = realweight + lut_graph[y_temp - 1][x_temp - 1][theta_temp];
	}
	return true;
}

bool preprocessedObservation::getprolut2map()
{
	ifstream fin;
	//initial lut_graph
	lut_graph = new uint8_t **[row];
	for (int i = 0; i < row; i++)
	{
		lut_graph[i] = new uint8_t *[col];
		for (int j = 0; j < col; j++) {
			lut_graph[i][j] = new uint8_t[row];
		}
	}
	//load txt uint_8t style
	fin.open("home/zhangjiwen/Desktop/lut_graph.txt");
	if (!fin.is_open())
	{
		cout << "fail to read ";
		return false;
	}
	int temp;
		for (int c = 0; c < pag; c++)
		{
			for (int a = 0; a < row; a++)
			{
				for (int b = 0; b < col; b++) 
				{
						fin >> temp;
						lut_graph[a][b][c] = temp;
				}
			}
		}
	fin.close();
	return true;
}

