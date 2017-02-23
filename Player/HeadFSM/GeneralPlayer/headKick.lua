------------------------------
-- Fix the head angle during approaching
------------------------------

module(..., package.seeall);

require('Body')
require('wcm')
require('mcm')
require('HeadTransform');

t0 = 0;

-- follow period
timeout = Config.fsm.headKick.timeout;
tLost = Config.fsm.headKick.tLost;
pitch0 = Config.fsm.headKick.pitch0;
xMax = Config.fsm.headKick.xMax;
yMax = Config.fsm.headKick.yMax;

function entry()
  print("Head SM:".._NAME.." entry");
  t0 = Body.get_time();
  kick_dir=wcm.get_kick_dir();
end

function update()
  pitchBias =  mcm.get_headPitchBias();--robot specific head bias

  local t = Body.get_time();
  local ball = wcm.get_ball();

  if ball.x<xMax and math.abs(ball.y)<yMax then
     Body.set_head_command({0, pitch0-pitchBias});
	 Body.set_para_headpos(vector.new({0, pitch0-pitchBias}));--123456頭部
     Body.set_state_headValid(1);--123456頭部
  else
   local yaw, pitch = HeadTransform.ikineCam(ball.x, ball.y, 0.03);
   --local currentYaw = Body.get_sensor_headpos()[1];--123456
   --local currentPitch = Body.get_sensor_headpos()[2];--123456
   local currentYaw = Body.get_sensor_headpos()[2];	--b51
   local currentPitch = Body.get_sensor_headpos()[1];	--b51
   local p = 0.3;
   yaw = currentYaw + p*(yaw - currentYaw);
   pitch = currentPitch + p*(pitch - currentPitch);
   Body.set_head_command({yaw, pitch});
   Body.set_para_headpos(vector.new({yaw, pitch}));--123456頭部
   Body.set_state_headValid(1);--123456頭部
  end

  if (t - ball.t > tLost) then
    return "ballLost";
  end
  if (t - t0 > timeout) then
    return "timeout";
  end
end

function exit()
end
