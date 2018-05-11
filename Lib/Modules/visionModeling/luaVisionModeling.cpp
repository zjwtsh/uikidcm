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

/*
#include "color_count.h"
#include "block_bitor.h"
#include "ConnectRegions.h"
#include "lua_color_stats.h"
#include "lua_goal_posts.h"
#include "lua_goal_posts_white.h"
#include "lua_field_lines.h"
#include "lua_field_spots.h"
#include "lua_field_occupancy.h"
#include "lua_robots.h"
*/
#include "lua_accumulate_ball.h"
#include "ballModelingObject.h"

static ballModelingObject * lua_checkvm(lua_State *L, int narg) {
  void *ud = luaL_checkudata(L, narg, "vm_mt");
  luaL_argcheck(L, *(ballModelingObject **)ud != NULL, narg, "invalid ball modeling object");
  return *(ballModelingObject **)ud;
}

static int lua_vm_resetBallModeling(lua_State *L) {
	ballModelingObject *vm = lua_checkvm(L, 1);
	vm->clearBootstrap();

	return 0;
}

static int lua_vm_modelingBall(lua_State *L) {
	ballModelingObject *vm = lua_checkvm(L, 1);

	//parameters transfered from lua
	uint8_t *x = (uint8_t *) lua_touserdata(L, 1);
	if ((x == NULL) || !lua_islightuserdata(L, 1)) {
		return luaL_error(L, "Input image not light user data");
	}
	int mx = luaL_checkint(L, 2);
	int nx = luaL_checkint(L, 3);
	double headPitch = luaL_checknumber(L, 4);

	vm->RunOneStep(x, mx, nx, headPitch);
	/*
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
	*/

	return 1;
}

static int lua_vm_modelingSelf(lua_State *L) {
	return 1;
}

static int lua_vm_create(lua_State *L) {
	ballModelingObject **ud = (ballModelingObject **)
		lua_newuserdata(L, sizeof(ballModelingObject *));
	
	*ud = new ballModelingObject;
	luaL_getmetatable(L, "vm_mt");
	lua_setmetatable(L, -2);

	return 1;
}

static int lua_vm_delete(lua_State *L) {
	ballModelingObject *vm = lua_checkvm(L, 1);
	delete vm;
	return 0;
}

static int lua_vm_destroy(lua_State *L) {
	return 0;
}

static int lua_vm_tostring(lua_State *L) {
	return 0;
}

static const struct luaL_reg visionModeling_lib [] = {
  {"new", lua_vm_create},
  {"destroy", lua_vm_destroy},
  {NULL, NULL}
};

static const struct luaL_reg visionModeling_methods[] = {
  {"__gc", lua_vm_delete},
  {"__tostring", lua_vm_tostring},
  {"ResetModelingBall", lua_vm_resetBallModeling},
  {"modelingBall", lua_vm_modelingBall},
  {"modelingSelf", lua_vm_modelingSelf},
  {NULL, NULL}
};

extern "C"
int luaopen_VisionModeling (lua_State *L) {
  luaL_newmetatable(L, "vm_mt");

  // OO access: mt.__index = mt
  lua_pushvalue(L, -1);
  lua_setfield(L, -2, "__index");

  //register other method to OO access port
  luaL_register(L, NULL, visionModeling_methods);
  
  //register the whole module
  luaL_register(L, "VisionModeling", visionModeling_lib);

  return 1;
}

