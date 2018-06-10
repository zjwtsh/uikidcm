module(... or "", package.seeall)

-- this module is used to facilitate interactive debuging

cwd = '.';

uname = io.popen('uname -s')
system = uname:read();

computer = os.getenv('COMPUTER') or system;
package.cpath = cwd.."/Lib/?.so;"..package.cpath;

package.path = cwd.."/Util/?.lua;"..package.path;
package.path = cwd.."/Config/?.lua;"..package.path;
package.path = cwd.."/Lib/?.lua;"..package.path;
package.path = cwd.."/Lib/Util/?.lua;"..package.path;
package.path = cwd.."/Dev/?.lua;"..package.path;
package.path = cwd.."/Motion/?.lua;"..package.path;
package.path = cwd.."/Motion/keyframes/?.lua;"..package.path;
package.path = cwd.."/Vision/?.lua;"..package.path;
package.path = cwd.."/World/?.lua;"..package.path;

--[[
require('serialization')
require('string')
require('vector')
require('getch')
require('util')
require('unix')
require('cutil')
require('shm')
]]--

require('VisionModeling')
require('vector')

local function initLineDetect()
		
		local line={};
		line.name = "line";
		line.detect = 1;

		if (line.detect == 1) then
			local v1x = vector.zeros(12);
			local v1y = vector.zeros(12);
			local v2x = vector.zeros(12);
			local v2y = vector.zeros(12);
			local real_length = vector.zeros(12);
			local endpoint11 = vector.zeros(12);
			local endpoint12 = vector.zeros(12);
			local endpoint21 = vector.zeros(12);
			local endpoint22 = vector.zeros(12);
			local xMean = vector.zeros(12);
			local yMean = vector.zeros(12);

			line.nLines = 1;
			--[[
			v1x = vcm.get_line_v1x();
			v1y = vcm.get_line_v1y();
			v2x = vcm.get_line_v2x();
			v2y = vcm.get_line_v2y();
			real_length = vcm.get_line_real_length();
			endpoint11 = vcm.get_line_endpoint11();
			endpoint12 = vcm.get_line_endpoint12();
			endpoint21 = vcm.get_line_endpoint21();
			endpoint22 = vcm.get_line_endpoint22();
			xMean = vcm.get_line_xMean();
			yMean = vcm.get_line_yMean();
			--]]

			line.v = {};
			line.endpoint = {};
			line.meanpoint = {};

			for i=1,line.nLines do
				line.v[i] = {};
				--line.v[i][1] = {};
				--line.v[i][2] = {};

				line.v[i][1] = 1 --v1x[i];
				line.v[i][2] = 2 --v1y[i];
				line.v[i][3] = 3 --v2x[i];
				line.v[i][4] = 4 --v2y[i];
			
				line.endpoint[i] = {};

				line.endpoint[i][1] = endpoint11[i];
				line.endpoint[i][3] = endpoint12[i];
				line.endpoint[i][2] = endpoint21[i];
				line.endpoint[i][4] = endpoint22[i];

				line.meanpoint[i] = {}; 
				line.meanpoint[i][1] = xMean[i];
				line.meanpoint[i][2] = yMean[i];

			end
		end
		return line;
end

ballObject = VisionModeling.new();
ballObject:init();
result = initLineDetect();
ballObject:runstep(result);


