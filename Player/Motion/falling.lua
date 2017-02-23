module(..., package.seeall);

require('Body')
require('walk')

t0 = 0;
timeout = Config.falling_timeout or 0.3;

isReset = false;--123456復位標誌

qLArmFront = vector.new({45,9,-135})*math.pi/180;
qRArmFront = vector.new({45,-9,-135})*math.pi/180;
realVel = vector.new({0,0,0});

function entry()
  print(_NAME.." entry");

  -- relax all the joints while falling
  Body.set_body_hardness(0);
  ----------------------------------------掉電123456

--[[
  --Ukemi motion (safe fall)
  local imuAngleY = Body.get_sensor_imuAngle(2);
  if (imuAngleY > 0) then --Front falling 
print("UKEMI FRONT")
    Body.set_larm_hardness({0.6,0,0.6});
    Body.set_rarm_hardness({0.6,0,0.6});
    Body.set_larm_command(qLArmFront);
    Body.set_rarm_command(qRArmFront);
  else
  end
--]]

  t0 = Body.get_time();
  Body.set_syncread_enable(1); --OP specific
  walk.stance_reset();--reset current stance
  --123456是否需要延時？？
	Body.set_para_velocity(vector.new({0,0,0}));
	Body.set_state_gaitValid(1);-----------------------------調用運動復位再延時一會兒123456
  Body.set_state_torqueEnable(0);-----------------------------123456摔倒時舵機斷電
  --unix.usleep(4000000);
  unix.sleep(1.0);    --tse
end

function update()
  local t = Body.get_time();
  -- set the robots command joint angles to thier current positions
  --  this is needed to that when the hardness is re-enabled
  gaitReset = Body.get_state_gaitReset();-----------------------------調用運動復位再延時一會兒123456
  isComplete = Body.get_state_gaitResetPending();
  if (t-t0 > timeout and isReset == false) then
    Body.set_state_torqueEnable(1);
    unix.sleep(1.0);
    	Body.set_state_gaitReset(1);
    	unix.usleep(500000);
	isReset = true;
  elseif (isReset == true and gaitReset[1] == 0 and isComplete[1] == 0) then
	isReset = false;-----------------------------調用運動復位再延時一會兒123456
    return "done"
  end
end

function exit()
  local qSensor = Body.get_sensor_position();
  Body.set_actuator_command(qSensor);end

