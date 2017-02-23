package.path = "../Config/World/?.lua;" .. package.path;
package.path = "../Util/?.lua;" .. package.path;
require('Config_OP_World')
require('vector')
Lcorner = Config_OP_World.world.Lcorner;

corner_lut_a = {};
corner_lut_r = {};
rCorner = vector.zeros(#Lcorner);
aCorner = vector.zeros(#Lcorner);
x0 = -3.05;
y0 = -2.05;

for ix = 1,60 do
	x = x0 + ix*.1;
	corner_lut_a[ix] = {};
	corner_lut_r[ix] = {};
	for iy = 1,40 do
		y = y0 + iy*.1;
		corner_lut_a[ix][iy] = {};
		corner_lut_r[ix][iy] = {};
		for ipos = 1,#Lcorner do
			dx = Lcorner[ipos][1]-x;
			dy = Lcorner[ipos][2]-y;
			rCorner[ipos] = math.sqrt(dx^2+dy^2);
			aCorner[ipos] = math.atan2(dy,dx);
			corner_lut_a[ix][iy] = aCorner;
			corner_lut_r[ix][iy] = rCorner;
		end
	end
--	print(aCorner);
end
