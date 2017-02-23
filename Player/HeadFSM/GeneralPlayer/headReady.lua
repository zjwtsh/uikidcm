module(..., package.seeall);
--SJ: camera IK based constant sweeping

require('Body')

t0 = 0;
dist = Config.fsm.headReady.dist;
yawMag = Config.head.yawMax;
tScan = Config.fsm.headReady.tScan;

function entry()
  print(_NAME.." entry");

  t0 = Body.get_time();
  --headAngles = Body.get_sensor_headpos();--123456
  headAngles = {Body.get_sensor_headpos()[2],Body.get_sensor_headpos()[1]};	--b51
  if (headAngles[1] > 0) then
    direction = 1;
  else
    direction = -1;
  end

  -- use top camera only
--  vcm.set_camera_command(0);
end

function update()

   local t = Body.get_time();
   local ph = (t-t0)/tScan;
   local height=vcm.get_camera_height();

--IK based horizon following
   local yaw0 = direction*(ph-0.5)*2*yawMag;
   local yaw, pitch =HeadTransform.ikineCam(
	dist*math.cos(yaw0),dist*math.sin(yaw0), height);

--ignore headangle limit for testing
   local yaw, pitch =HeadTransform.ikineCam0(
	dist*math.cos(yaw0),dist*math.sin(yaw0), height);
	pitch = pitch + 5*math.pi/180;
   Body.set_head_command({yaw, pitch});
  
   Body.set_para_headpos(vector.new({yaw, pitch}));--123456�^��
   Body.set_state_headValid(1);--123456�^��
  
   if (t - t0 > tScan) then
    return 'done'
   end
end

function exit()
end
