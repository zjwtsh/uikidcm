module(... or "", package.seeall)
package.cpath = './Lib/?.so;' .. package.cpath  
require('unix');
require('main_op');

while 1 do 
  tDelay = 0.005*1E6;
  main_op.update();
  unix.usleep(tDelay);
end

