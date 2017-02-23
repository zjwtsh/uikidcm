module(... or '', package.seeall)
cwd = '.';
package.cpath = cwd .. '/Lib/?.so;' .. package.path;
package.path = cwd .. '/Util/?.lua;' .. package.path;

require("unix");
require("util");

count = 0;
t0 = unix.time();

while 1 do
	a = {.6, 1.2}
	math.sqrt(a[1],a[2]);
	count = count +1;
	if (count%1000 == 0) then
		t1 = unix.time();
		print(t1-t0);
		t0 = t1;
	end
end
