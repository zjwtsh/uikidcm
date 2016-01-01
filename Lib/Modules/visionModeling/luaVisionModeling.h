#ifndef luaVisionModeling_h_DEFINED
#define luaVisionMOdeling_h_DEFINED

extern "C"
{
#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
}


extern "C"
int luaopen_VisionModeling (lua_State *L);

#endif
