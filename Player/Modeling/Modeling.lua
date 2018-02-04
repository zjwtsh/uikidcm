module(..., package.seeall);

require('carray');
require('vector');
require('Config');

require('ImageProc');
require('HeadTransform');

require('vcm');
require('mcm');
require('Body');

black_depth = Config.vision.black_depth or 8
black_width = Config.vision.black_width or 8
lut_stage = Config.vision.lut_stage or 64
use_arbitrary_ball = Config.vision.use_arbitrary_ball or false;

require('Camera');
require('Detection');

if (Config.camera.width ~= Camera.get_width()
    or Config.camera.height ~= Camera.get_height()) then
  print('Camera width/height mismatch');
  print('Config width/height = ('..Config.camera.width..', '..Config.camera.height..')');
  print('Camera width/height = ('..Camera.get_width()..', '..Camera.get_height()..')');
  error('Config file is not set correctly for this camera. Ensure the camera width and height are correct.');
end

vcm.set_image_width(Config.camera.width);
vcm.set_image_height(Config.camera.height);

-- image capturing from the camera
camera = {};
camera.width = Camera.get_width();
camera.height = Camera.get_height();
camera.npixel = camera.width*camera.height;
camera.image = Camera.get_image();
camera.status = Camera.get_camera_status();
camera.switchFreq = Config.camera.switchFreq;
camera.ncamera = Config.camera.ncamera;

-- Initialize the Labeling
labelA = {};
-- labeled image is 1/4 the size of the original
labelA.m = camera.width/2;
labelA.n = camera.height/2;
labelA.npixel = labelA.m*labelA.n;

scaleB = Config.vision.scaleB;
labelB = {};
labelB.m = labelA.m/scaleB;
labelB.n = labelA.n/scaleB;
labelB.npixel = labelB.m*labelB.n;
vcm.set_image_scaleB(Config.vision.scaleB);
print('Vision LabelA size: ('..labelA.m..', '..labelA.n..')');
print('Vision LabelB size: ('..labelB.m..', '..labelB.n..')');

--update new value to avoid direct writing of different tags
colorOrange = Config.color.orange;
colorYellow = Config.color.yellow;
colorCyan = Config.color.cyan;
colorField = Config.color.field;
colorWhite = Config.color.white;

-- debugging settings
vcm.set_debug_enable_shm_copy(Config.vision.copy_image_to_shm);
vcm.set_debug_store_goal_detections(Config.vision.store_goal_detections);
vcm.set_debug_store_ball_detections(Config.vision.store_ball_detections);
vcm.set_debug_store_all_images(Config.vision.store_all_images);

-- Timing
count = 0;
lastImageCount = {0,0};
t0 = unix.time();

function entry()
  -- Temporary value.. updated at body FSM at next frame
  vcm.set_camera_bodyHeight(Config.walk.bodyHeight);
	-- vcm.set_camera_bodyTilt(0);
  vcm.set_camera_height(Config.walk.bodyHeight+Config.head.neckZ);
	vcm.set_camera_ncamera(Config.camera.ncamera);

  -- Start the HeadTransform machine
  HeadTransform.entry();

  -- Initiate Detection
  Detection.entry();

  -- Load the lookup table
	-- preprocessing of lut for colored ball instead of the obsolete orange ball
	if(use_arbitrary_ball) then
		print('loading environment lut: '..Config.camera.lut_file);
		camera.lut = load_general_lut(Config.camera.lut_file);

		print('loading ball lut: '..Config.camera.lut_ball_file);
		camera.lut_ball = load_general_lut(Config.camera.lut_ball_file);
		--writeSplittedBallLut('ballLut.txt', camera.lut_ball)

		print('creating splitted ball lut');
		camera.splittedBallLut = split_ball_lut(camera.lut,camera.lut_ball);
		--writeSplittedBallLut('splittedLut.txt', camera.splittedBallLut)
		--print('exiting the test routine')
		--os.exit()

		print('temp test for transfering of field color from ball lut to env lut');
		camera.lut = add_ballfield_lut(camera.lut,camera.lut_ball);
		--writeSplittedBallLut('envLut.txt', camera.lut)
		--print('exiting the test routine')
		--os.exit()
	else
		camera.lut = load_general_lut(Config.camera.lut_file);
	end

  camera_init();

  -- in default, use prelearned colortable
  vcm.set_image_learn_lut(0);
end

function camera_init()
  for c=1,Config.camera.ncamera do 
    Camera.select_camera(c-1);
    for i,auto_param in ipairs(Config.camera.auto_param) do
      print('Camera '..c..': setting '..auto_param.key..': '..auto_param.val[c]);
      Camera.set_param(auto_param.key, auto_param.val[c]);
      unix.usleep(100000);
      print('Camera '..c..': set to '..auto_param.key..': '..Camera.get_param(auto_param.key));
    end   
    for i,param in ipairs(Config.camera.param) do
      print('Camera '..c..': setting '..param.key..': '..param.val[c]);
      Camera.set_param(param.key, param.val[c]);
      unix.usleep(10000);
      print('Camera '..c..': set to '..param.key..': '..Camera.get_param(param.key));
    end
  end
end

function update()
  tstart = unix.time();
  
  headAngles = {Body.get_sensor_headpos()[2],Body.get_sensor_headpos()[1]};	--b51
 
  -- get image from camera
  camera.image = Camera.get_image();
  local status = Camera.get_camera_status();
  if status.count ~= lastImageCount[status.select+1] then
    lastImageCount[status.select+1] = status.count;
  else
    return false; 
  end

  -- Add timer measurements
  count = count + 1;
	
	-- HeadTransform.update(status.select, headAngles, compensateY);
  HeadTransform.update(status.select, headAngles);

	--SJ: Camera image keeps changing
	--So copy it here to shm, and use it for all vision process
  vcm.set_image_yuyv(camera.image);

  if camera.image == -2 then
    print "Re-enqueuing of a buffer error...";
    os.exit()
  end

  -- perform the initial labeling
	if(use_arbitrary_ball) then
		labelA.data, labelA.dataBall  = ImageProc.yuyv_to_label_ball(vcm.get_image_yuyv(),
																					carray.pointer(camera.lut),
																					carray.pointer(camera.splittedBallLut),
																					camera.width/2,
																					camera.height);
	else
		labelA.data  = ImageProc.yuyv_to_label(vcm.get_image_yuyv(),
																					carray.pointer(camera.lut),
																					camera.width/2,
																					camera.height);
	end
	--print('exiting the test routine')
	--os.exit()

  -- determine total number of pixels of each color/label
  colorCount = ImageProc.color_count(labelA.data, labelA.npixel);

  -- bit-or the segmented image
  labelB.data = ImageProc.block_bitor(labelA.data, labelA.m, labelA.n, scaleB, scaleB);

  update_shm(status, headAngles)

  vcm.refresh_debug_message();
  Detection.update();
  vcm.refresh_debug_message();

  -- switch camera
  local cmd = vcm.get_camera_command();
  if (cmd == -1) then
    if (count % camera.switchFreq == 0) then
       Camera.select_camera(1-Camera.get_select()); 
    end
  else
    if (cmd >= 0 and cmd < camera.ncamera) then
      Camera.select_camera(cmd);
    else
      print('WARNING: attempting to switch to unkown camera select = '..cmd);
    end
  end

  return true;
end

function update_shm(status, headAngles)
  -- Update the shared memory
  -- Shared memory size argument is in number of bytes

  if vcm.get_debug_enable_shm_copy() == 1 then
    if ((vcm.get_debug_store_all_images() == 1)
      or (ball.detect == 1
          and vcm.get_debug_store_ball_detections() == 1)
      or ((goalCyan.detect == 1 or goalYellow.detect == 1)
          and vcm.get_debug_store_goal_detections() == 1)) then

      if vcm.get_camera_broadcast() > 0 then --Wired monitor broadcasting
	      if vcm.get_camera_broadcast() == 1 then
	    --Level 1: 1/4 yuyv, labelB
          vcm.set_image_yuyv3(ImageProc.subsample_yuyv2yuyv(
          vcm.get_image_yuyv(),
	        camera.width/2, camera.height,4));
          vcm.set_image_labelB(labelB.data);
        elseif vcm.get_camera_broadcast() == 2 then
	    --Level 2: 1/2 yuyv, labelA, labelB
          vcm.set_image_yuyv2(ImageProc.subsample_yuyv2yuyv(
          vcm.get_image_yuyv(),
          camera.width/2, camera.height,2));
          vcm.set_image_labelA(labelA.data);
          vcm.set_image_labelB(labelB.data);
	      else
	    --Level 3: 1/2 yuyv
          vcm.set_image_yuyv2(ImageProc.subsample_yuyv2yuyv(
          vcm.get_image_yuyv(),
          camera.width/2, camera.height,2));
	      end

	    elseif vcm.get_camera_teambroadcast() > 0 then --Wireless Team broadcasting
          --Only copy labelB
          vcm.set_image_labelB(labelB.data);
      end
    end
  end

  vcm.set_image_select(status.select);
  vcm.set_image_count(status.count);
  vcm.set_image_time(status.time);
  vcm.set_image_headAngles(headAngles);
  vcm.set_image_horizonA(HeadTransform.get_horizonA());
  vcm.set_image_horizonB(HeadTransform.get_horizonB());
  vcm.set_image_horizonDir(HeadTransform.get_horizonDir())

  update_shm_fov();
end

function update_shm_fov()
  --This function projects the boundary of current labeled image

  local fovC={Config.camera.width/2,Config.camera.height/2};
  local fovBL={0,Config.camera.height};
  local fovBR={Config.camera.width,Config.camera.height};
  local fovTL={0,0};
  local fovTR={Config.camera.width,0};

  vcm.set_image_fovC(vector.slice(HeadTransform.projectGround(
 	  HeadTransform.coordinatesA(fovC,0.1)),1,2));
  vcm.set_image_fovTL(vector.slice(HeadTransform.projectGround(
 	  HeadTransform.coordinatesA(fovTL,0.1)),1,2));
  vcm.set_image_fovTR(vector.slice(HeadTransform.projectGround(
 	  HeadTransform.coordinatesA(fovTR,0.1)),1,2));
  vcm.set_image_fovBL(vector.slice(HeadTransform.projectGround(
 	  HeadTransform.coordinatesA(fovBL,0.1)),1,2));
  vcm.set_image_fovBR(vector.slice(HeadTransform.projectGround(
 	  HeadTransform.coordinatesA(fovBR,0.1)),1,2));
end

function exit()
  HeadTransform.exit();
end

function writeSplittedBallLut(fname,lutContents)
  local cwd = unix.getcwd();
  cwd = cwd.."/Data/";

  local f = io.open(cwd..fname, "w");
  assert(f, "Could not open lut file");

	local lutLength = #lutContents;
  for i = 1, lutLength do
    f:write(lutContents[i], ",");
  end

	f:write("\n");
	
end

function load_general_lut(fname)
	local ta = carray.new('c',262144)

  local cwd = unix.getcwd();
  if string.find(cwd, "WebotsController") then
    cwd = cwd.."/Player";
  end
  cwd = cwd.."/Data/";

  local f = io.open(cwd..fname, "r");
  assert(f, "Could not open lut file");
  local s = f:read("*a");

	print('raw file length ='.. string.len(s))
  for i = 1,string.len(s) do
    ta[i] = string.byte(s,i,i);
  end
	
	return ta;
end

function isAlsoBlackColor(y,u,v)
	uvRange = {(lut_stage-black_width)/2,(lut_stage+black_width)/2}

	if((u-uvRange[1])*(u-uvRange[2])<0 and
			(v-uvRange[1])*(v-uvRange[2])<0 and
			y<black_depth) then
		--print("black color found in ball", y, u, v)
		return true;
	end
	return false;
end

function add_ballfield_lut(envLut, ballLut)
  local ta = carray.new('c', 262144);

	for i = 1, 64 do
    for j = 1, 64 do
      for k = 1, 64 do

				local ind = (i-1)*64*64 + (j-1)*64 + k;
				ta[ind] = envLut[ind];

				if(ta[ind] == 8) then
					ta[ind] = 0
				end

				if (ballLut[ind] == 8 and ta[ind] == 0) then
					ta[ind] = ballLut[ind];
				end

      end
    end
  end

  return ta;
end

function split_ball_lut(envLut, ballLut)
  local ta = carray.new('c', 262144);

	for i = 1, 64 do
    for j = 1, 64 do
      for k = 1, 64 do
				local ind = (i-1)*64*64 + (j-1)*64 + k;
				--ta[ind] = ballLut[ind];
				if (ballLut[ind]==1) then
					if(envLut[ind]==2 or envLut[ind]==16) then
						ta[ind] = 4
					elseif(isAlsoBlackColor(i,j,k)) then
						ta[ind] = 2
					else
						ta[ind] = 1
					end
				end
      end
    end
  end

	print("splitted lut for the ball is")
	print(ta)

  return ta;
end

