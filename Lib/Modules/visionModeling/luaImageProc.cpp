/* 
   Lua interface to object modeling and self localization Processing utilities.
	 Note there is no independant vision and world module since then.

   To compile on Mac OS X:
   g++ -arch i386 -o luaImageUtil.dylib -bundle -undefined dynamic_lookup luaImageUtil.cpp -lm
 */

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
#include <math.h>
#include <vector>
#include <string>
#include <algorithm>
#include <iostream>

#include "color_count.h"
#include "block_bitor.h"
#include "ConnectRegions.h"

#include "lua_color_stats.h"
#include "lua_goal_posts.h"
#include "lua_goal_posts_white.h"
#include "lua_field_lines.h"
#include "lua_field_spots.h"
#include "lua_field_occupancy.h"
#include "lua_accumulate_ball.h"
#include "lua_robots.h"

static int lua_visionModeling_ball(lua_State *L) {
  static std::vector<Candidate> ballCandidates;
	AccumulateParaIn paraIn;

  uint8_t *x = (uint8_t *) lua_touserdata(L, 1);
  if ((x == NULL) || !lua_islightuserdata(L, 1)) {
    return luaL_error(L, "Input image not light user data");
  }
  int mx = luaL_checkint(L, 2);
  int nx = luaL_checkint(L, 3);
  double headPitch = luaL_checknumber(L, 4);

	paraIn.cameraTilt = headPitch;
	paraIn.cameraAngleSpead = 50*M_PI/180;
	paraIn.physicalRadiusOfBall = 90;
	paraIn.horizonLimit = 5*M_PI/180;
	paraIn.noiseRate = 0.3;
	paraIn.radiusRate = 1.2;
	int nball = lua_accumulate_ball(ballCandidates, x, mx, nx, paraIn);

  if (nball <= 0) {
    return 0;
  }

  lua_createtable(L, nball, 0);
  for (int i = 0; i < nball; i++) {
    lua_createtable(L, 0, 5);

    lua_pushstring(L, "blCntr");
    lua_pushnumber(L, ballCandidates[i].blCntr);
    lua_settable(L, -3);

    lua_pushstring(L, "wtCntr");
    lua_pushnumber(L, ballCandidates[i].wtCntr);
    lua_settable(L, -3);

    lua_pushstring(L, "bkCntr");
    lua_pushnumber(L, ballCandidates[i].bkCntr);
    lua_settable(L, -3);

    lua_pushstring(L, "radiusRate");
    lua_pushnumber(L, ballCandidates[i].evaluation);
    lua_settable(L, -3);

    // boundingBox field
    lua_pushstring(L, "boundingBox");
    lua_createtable(L, 4, 0);
    lua_pushnumber(L, ballCandidates[i].bBox[0].x);
    lua_rawseti(L, -2, 1);
    lua_pushnumber(L, ballCandidates[i].bBox[1].x);
    lua_rawseti(L, -2, 2);
    lua_pushnumber(L, ballCandidates[i].bBox[0].y);
    //lua_pushnumber(L, props[i].minJ + rowOffset);
    lua_rawseti(L, -2, 3);
    lua_pushnumber(L, ballCandidates[i].bBox[1].y);
    //lua_pushnumber(L, props[i].maxJ + rowOffset);
    lua_rawseti(L, -2, 4);
    lua_settable(L, -3);

    lua_rawseti(L, -2, i+1);
  }
  return 1;
}

static int lua_resetModeling_ball(lua_State *L) {
	return 1;
}

static int lua_visionModeling_self(lua_State *L) {
	return 1;
}

static const struct luaL_reg visionModeling_lib [] = {
  {"ResetModelingBall", lua_resetModeling_ball},
  {"modelingBall", lua_visionModeling_ball},
  {"modelingSelf", lua_visionModeling_self},
  {NULL, NULL}
};

extern "C"
int luaopen_VisionModeling (lua_State *L) {
  luaL_register(L, "VisionModeling", visionModeling_lib);

  return 1;
}


