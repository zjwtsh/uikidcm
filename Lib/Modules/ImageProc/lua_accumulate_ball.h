#ifndef lua_accumulate_ball_h_DEFINED
#define lua_accumulate_ball_h_DEFINED

#include <stdint.h>

typedef struct
{
  int x;
  int y;
}Array2D;

typedef struct
{
  uint8_t colorTag;
  int colorCount;
  Array2D bBox[2];
}Cluster;

typedef struct
{
  Array2D bBox[2];
  int blCntr;
  int wtCntr;
  int bkCntr;
  float evaluation;
}Candidate;

typedef struct 
{
	double cameraTilt;
	double cameraAngleSpead;
	double physicalRadiusOfBall;
}AccumulateParaIn;

int lua_accumulate_ball(std::vector <Candidate> &ballCandidates, uint8_t *label, int width, int height, AccumulateParaIn &paraIn);

#endif
