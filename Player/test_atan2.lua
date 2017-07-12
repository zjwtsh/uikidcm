module(... or '', package.seeall);

cwd = '.';
package.cpath = cwd .. '/Lib/?.so;' .. package.path;
package.path = cwd .. '/Util/?.lua;' .. package.path;

require('unix');
require('util');
require('Transform')

count = 0;
t0 = unix.time();

a = Transform.eye();
a[1] = vector.new({1, 2, 3, 4});
a[2] = vector.new({2, 3, 5, 6});
a[3] = vector.new({2, 2, 3, 1});
a[4] = vector.new({2, 4, 6, 7});
b = Transform.eye();
v = Transform.eye();
v[1] = vector.new({1, 2, 3, 4});
v[2] = vector.new({3, 4, 5, 5});
v[3] = vector.new({5, 6, 7, 8});
v[4] = vector.new({9, 8, 5, 1});
b = a*v;
print(b[1][1].." "..b[1][2].." "..b[1][3].." "..b[1][4]);
print(b[2][1].." "..b[2][2].." "..b[2][3].." "..b[2][4]);
print(b[3][1].." "..b[3][2].." "..b[3][3].." "..b[3][4]);
print(b[4][1].." "..b[4][2].." "..b[4][3].." "..b[4][4]);
