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

static int lua_vm_resetModeling(lua_State *L) {
	ballModelingObject *vm = lua_checkvm(L, 1);
	vm->clearBootstrap();

	return 0;
}

static int lua_vm_initModeling(lua_State *L) {
	ballModelingObject *vm = lua_checkvm(L, 1);
	
	MatrixWrapper::ColumnVector initState(3);
	initState(1) = 0.0;
	initState(2) = 0.0;
	initState(3) = 0.0;
	vm->InitializeBootStrapFilter(initState);

	return 0;
}

static int lua_vm_updateBallObservation(lua_State *L) {
	//1. get the user object
	ballModelingObject *vm = lua_checkvm(L, 1);
	//2. convert the lua table into cpp structure

	//3. call update function of vm
	return 0;

}

static int lua_vm_runStep(lua_State *L) {
	ballModelingObject *vm = lua_checkvm(L, 1);

	if(!vm->ExtractLineInfoByLua(L,-1))
		return 0;

	//change the interface for line observation
	vm->RunOneStep();
	return 1;
}

static int lua_vm_create(lua_State *L) {
	std::cout << "starting to create the objectModeling according to input: ball or robot" << std::endl;
	ballModelingObject **ud = (ballModelingObject **)
		lua_newuserdata(L, sizeof(ballModelingObject *));
	
	*ud = new ballModelingObject;
	luaL_getmetatable(L, "vm_mt");
	lua_setmetatable(L, -2);

	return 1;
}

static int lua_vm_delete(lua_State *L) {
	std::cout << "the ballModelingObject will be deleted " <<std::endl;
	ballModelingObject *vm = lua_checkvm(L, 1);
	delete vm;
	return 0;
}

static int lua_vm_destroy(lua_State *L) {
	//use the module to delete object, ommitted here
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
  {"init", lua_vm_initModeling},
  {"reset", lua_vm_resetModeling},
  {"runstep", lua_vm_runStep},
  {"updateBallObservation", lua_vm_updateBallObservation},
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

