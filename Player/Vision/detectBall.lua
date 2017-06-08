module(..., package.seeall);

require('Config');      -- For Ball and Goal Size
require('ImageProc');
require('HeadTransform');       -- For Projection
require('Vision');
require('Body');
require('shm');
require('vcm');
require('mcm');
require('Detection');
require('Debug');

-- Define Color
colorOrange = Config.color.orange;
colorYellow = Config.color.yellow;
colorCyan = Config.color.cyan;
colorField = Config.color.field;
colorWhite = Config.color.white;

diameter = Config.vision.ball.diameter;
th_min_color=Config.vision.ball.th_min_color;
th_min_color2=Config.vision.ball.th_min_color2;
th_max_color2=1200;
th_min_fill_rate=Config.vision.ball.th_min_fill_rate;
th_max_fill_rate=Config.vision.ball.th_max_fill_rate;
th_min_black_rate=0.1;
th_max_black_rate = 0.5;
th_min_black_area = 3;
max_distance = 3.5;
th_height_max=Config.vision.ball.th_height_max;
th_height_min = -0.2;
th_ground_boundingbox=Config.vision.ball.th_ground_boundingbox;
th_min_green1=Config.vision.ball.th_min_green1;
th_min_green2=Config.vision.ball.th_min_green2;
th_max_aspect_ratio = 1.7;
th_min_aspect_ratio = 0.5;

ball_check_for_ground = Config.vision.ball.check_for_ground;
check_for_field = Config.vision.ball.check_for_field or 0;
field_margin = Config.vision.ball.field_margin or 0;

th_headAngle = Config.vision.ball.th_headAngle or 30*math.pi/180;

function detect(color)
	
	t001 = unix.time();	--b51
--  enable_obs_challenge = Config.obs_challenge or 0;
--  if enable_obs_challenge == 1 then
--    colorCount = Vision.colorCount_obs;
--  else
    colorCount = Vision.colorCount;
--  end

  --headAngle = Body.get_head_position();
  headAngle = {Body.get_sensor_headpos()[2],Body.get_sensor_headpos()[1]};	--b51
  --print("headangle detectball:",headAngle[1]*180/math.pi, headAngle[2]*180/math.pi);
  local ball = {};
  ball.detect = 0;
--  vcm.add_debug_message(string.format("\nBall: pixel count: %d\n",
--	colorCount[color]));
  
--  print(string.format("\nBall: pixel count: %d\n",
--	      colorCount[color]));


  -- threshold check on the total number of ball pixels in the image
  if (colorCount[color] < th_min_color) then  	
--    vcm.add_debug_message("pixel count fail");
    return ball;  	
  end

  -- find connected components of ball pixels
--  if enable_obs_challenge == 1 then
--    ballPropsB = ImageProc.connected_regions_obs(Vision.labelB.data_obs, Vision.labelB.m, 
--                                              Vision.labelB.n, colorWhite);
--  else
    local ballPropsB = ImageProc.connected_regions(Vision.labelB.data, Vision.labelB.m, Vision.labelB.n, colorWhite);
--  end
--  util.ptable(ballPropsB);
--TODO: horizon cutout
-- ballPropsB = ImageProc.connected_regions(labelB.data, labelB.m, 
--	labelB.n, HeadTransform.get_horizonB(),color);

--  ballPropsB = Vision.labelBall;


  if (not ballPropsB or #ballPropsB == 0) then return ball; end

-- Check max 5 largest blobs 
  --for i=1,math.min(5,#ballPropsB) do
  for i=1, #ballPropsB do
--    vcm.add_debug_message(string.format(
--	"Ball: checking blob %d/%d\n",i,#ballPropsB));

    check_passed = true;
    ball.propsB = ballPropsB[i];
    ball.propsA = Vision.bboxStats(colorWhite, ballPropsB[i].boundingBox);
    ball.bboxA = Vision.bboxB2A(ballPropsB[i].boundingBox);
    
    local aspect_ratio = ball.propsA.axisMajor / ball.propsA.axisMinor;
    if (ball.propsA.axisMajor == 0) then aspect_ratio = ball.propsA.axisMajor / 0.00001 end

    local props_black = ImageProc.color_stats(Vision.labelA.data, Vision.labelA.m, Vision.labelA.n, colorOrange, ball.bboxA);
    local black_rate = props_black.area / ball.propsA.area;

    local fill_rate = (ball.propsA.area + props_black.area) / Vision.bboxArea(ball.propsA.boundingBox);
--    vcm.add_debug_message(string.format("Area:%d\nFill rate:%2f\n",
--       ball.propsA.area+props_black.area,fill_rate));

    if ball.propsA.area > th_max_color2 then
      check_passed = false;
    elseif ball.propsA.area < th_min_color2 then
      --Area check
      --vcm.add_debug_message("Area check fail\n");
      check_passed = false;
    elseif fill_rate < th_min_fill_rate then
      --Fill rate check
      check_passed = false;
    elseif ball.propsA.boundingBox[4] < HeadTransform.get_horizonA() then
      check_passed = false;
    elseif aspect_ratio > th_max_aspect_ratio then
      check_passed = false;
    elseif aspect_ratio < th_min_aspect_ratio then
      check_passed = false;
    elseif black_rate < th_min_black_rate then
      check_passed = false;
    elseif props_black.area < th_min_black_area then
        check_passed = false;
    else
      -- diameter of the area
      local dArea = math.sqrt((4/math.pi)*ball.propsA.area);
     -- Find the centroid of the ball
      local ballCentroid = ball.propsA.centroid;
      local scale = math.max(dArea/diameter, ball.propsA.axisMajor/diameter)
      
      --print("ballCentroid :"..ballCentroid[1],ballCentroid[2]);
      
      -- Coordinates of ball
      local scale = math.max(dArea/diameter, ball.propsA.axisMajor/diameter);
      v = HeadTransform.coordinatesA(ballCentroid, scale);
      --print("ballv"..v[1],v[2]);	--168
      v_inf = HeadTransform.coordinatesA(ballCentroid,0.1);
--      vcm.add_debug_message(string.format(
--	"Ball v0: %.2f %.2f %.2f\n",v[1],v[2],v[3]));

      if v[3] > th_height_max or v[3] < th_height_min then
        --Ball height check
--        vcm.add_debug_message("Height check fail\n");
        check_passed = false;
      end

      if check_passed then
        ball_dist_inf = math.sqrt(v_inf[1]*v_inf[1] + v_inf[2]*v_inf[2])
        height_th_inf = th_height_max + ball_dist_inf * math.tan(10*math.pi/180)
        if v_inf[3] > height_th_inf then        
           -- vcm:add_debug_message(string.format('Horizon check fail, %.2f>%.2f\n',v_inf[3],height_th_inf));
           check_passed = false;
        end

        pose=wcm.get_pose();
        posexya=vector.new( {pose.x, pose.y, pose.a} );
        ballGlobal = util.pose_global({v[1],v[2],0},posexya); 
        if ballGlobal[1]>Config.world.xMax * 2.0 or
           ballGlobal[1]<-Config.world.xMax* 2.0 or
           ballGlobal[2]>Config.world.yMax * 2.0 or
           ballGlobal[2]<-Config.world.yMax* 2.0 then
           if (v[1]*v[1] + v[2]*v[2] > max_distance*max_distance) then          
--             vcm:add_debug_message("On-the-field check fail\n");
             check_passed = false;
           end
        end

        local ball_dist = math.sqrt(v[1]*v[1] + v[2]*v[2])
        local height_th = th_height_max + ball_dist * math.tan(8*math.pi/180)
        if check_passed and v[3] > 0.3 then
             -- print('v3 is '..v[3])
             -- print('reached')
           -- vcm:add_debug_message(string.format('Failure: Ball Height Check Fail \n')); 
		     check_passed = false
        end

        if check_passed and v[3] > height_th then
          -- vcm:add_debug_message(string.format('Height check fail\n',v[3],height_th))
          check_passed = false; 
        end     

        if (ball_check_for_ground > 0)  then  -- ground check
          local vmargin=Vision.labelA.n-ballCentroid[2];
          local hmargin=Vision.labelA.m-ballCentroid[1];
          if (ballCentroid[1] > dArea) then
            local fieldBBox_left = {}
            fieldBBox_left[1] = ballCentroid[1] - .5*dArea + th_ground_boundingbox[1];
            fieldBBox_left[2] = ballCentroid[1] - .5*dArea;
            fieldBBox_left[3] = ballCentroid[2] - .5*dArea;
            fieldBBox_left[4]= ballCentroid[2] + .5*dArea;
            local left_area = Vision.bboxArea(fieldBBox_left);
          
            local fieldBBoxStats_left = ImageProc.color_stats(Vision.labelA.data,
                                  Vision.labelA.m, Vision.labelA.n, colorField, fieldBBox_left);
             
            if (fieldBBoxStats_left.area/left_area < th_min_green1) then
              --vcm.add_debug_message(string.format("Failure: left green check\n"));
				      check_passed = false;
            end
          else
            check_passed = false;
          end

          if (hmargin > dArea or (hmargin > dArea / 2)) and check_passed then 
            local fieldBBox_right = {}
            fieldBBox_right[1] = ballCentroid[1] + .5*dArea;
            fieldBBox_right[2] = ballCentroid[1] + .5*dArea + th_ground_boundingbox[2];
            fieldBBox_right[3] = ballCentroid[2] - .5*dArea;
            fieldBBox_right[4] = ballCentroid[2] + .5*dArea;
            local right_area = Vision.bboxArea(fieldBBox_right);
            local fieldBBoxStats_right = ImageProc.color_stats(Vision.labelA.data,
                                  Vision.labelA.m, Vision.labelA.n, colorField, fieldBBox_right);
             
            if (fieldBBoxStats_right.area/right_area < th_min_green1) then
              --vcm:add_debug_message(string.format("Failure: right green check\n"));
              check_passed = false;
            end
          else
            check_passed = false;
          end
            
          if ballCentroid[2] > dArea and check_passed then 
            local fieldBBox_top = {}
            fieldBBox_top[1] = ballCentroid[1] - .5*dArea;
            fieldBBox_top[2] = ballCentroid[1] + .5*dArea;
            fieldBBox_top[3] = ballCentroid[2] - .5*dArea + th_ground_boundingbox[3];
            fieldBBox_top[4] = ballCentroid[2] - .5*dArea;
            local top_green_area = Vision.bboxArea(fieldBBox_top);
         
            local fieldBBoxStats_top = ImageProc.color_stats(Vision.labelA.data, Vision.labelA.m, Vision.labelA.n, colorField, fieldBBox_top);
            if (fieldBBoxStats_top.area/top_green_area < th_min_green1) then
              --vcm:add_debug_message(string.format("Failure: top green check\n"));
				      check_passed = false;
            end
          else
            check_passed = false;
          end

        end -- End ground check
      end -- End check_pass check
    end --End all check
    if check_passed then break end
  end --End loop

  if not check_passed then
    return ball;
  end
  
  --SJ: Projecting ball to flat ground makes large distance error
  --We are using declined plane for projection

  --vMag =math.max(0,math.sqrt(v[1]^2+v[2]^2)-0.50);
  --bodyTilt = vcm.get_camera_bodyTilt();
----  print("BodyTilt:",bodyTilt*180/math.pi)
  --projHeight = vMag * math.tan(10*math.pi/180);


  --v=HeadTransform.projectGround(v,diameter/2-projHeight);

  --SJ: we subtract foot offset 
  --bc we use ball.x for kick alignment
  --and the distance from foot is important
  v[1]=v[1]-mcm.get_footX()

  ball_shift = Config.ball_shift or {0,0};
  --Compensate for camera tilt
  v[1]=v[1] + ball_shift[1];
  v[2]=v[2] + ball_shift[2];
  --Ball position ignoring ball size (for distant ball observation)
  v_inf=HeadTransform.projectGround(v_inf,diameter/2);
  v_inf[1]=v_inf[1]-mcm.get_footX()
  
  wcm.set_ball_v_inf({v_inf[1],v_inf[2]});  

  ball.v = v;
  --print("ball.v :"..v[1],v[2]);	--168
  ball.detect = 1;
  ball.r = math.sqrt(ball.v[1]^2 + ball.v[2]^2);
  
  -- How much to update the particle filter
  ball.dr = 0.25*ball.r;
  ball.da = 10*math.pi/180;

--  vcm.add_debug_message(string.format(
--	"Ball detected\nv: %.2f %.2f %.2f\n",v[1],v[2],v[3]));
--[[
  print(string.format(
	"Ball detected\nv: %.2f %.2f %.2f\n",v[1],v[2],v[3]));
--]]
	t003 = unix.time();
  return ball;
end

