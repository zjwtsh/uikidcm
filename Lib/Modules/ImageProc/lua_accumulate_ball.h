#ifndef lua_accumulate_ball_h_DEFINED
#define lua_accumulate_ball_h_DEFINED

#include <stdint.h>

typedef struct
{
  uint8_t x;
  uint8_t y;
}Array2D;

typedef struct
{
  uint8_t colorTag;
  int colorCount;
  std::vector<Array2D> bBox;
}Cluster;

typedef struct
{
  std::vector<Array2D> bBox;
  int blCntr;
  int wtCntr;
  int bkCntr;
  float evaluation;
}Candidate;

//static std::vector<Array2D> fourConn(4,{0, 0});

int lua_accumulate_ball(std::vector <Candidate> &ballCandidates, uint8_t *label, int width, int height);

#endif
