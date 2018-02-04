module(..., package.seeall);

require('Config');      -- For Ball and Goal Size
require('ImageProc');
require('HeadTransform');       -- For Projection
require('Vision');
require('Body');
require('shm');
require('Detection');

-- Define Color
colorOrange = Config.color.orange;
colorYellow = Config.color.yellow;
colorCyan = Config.color.cyan;
colorField = Config.color.field;
colorWhite = Config.color.white;

diameter = Config.vision.ball.diameter; -- 0.14

function getCheckBoxWithOffset(target, offset)
	ret = target;
	ret[1] = ret[1] - offset;
	ret[2] = ret[2] + offset;
	ret[3] = ret[3] - offset;
	ret[4] = ret[4] + offset;
	return ret;
end

function modeling()
	headAngle = {Body.get_sensor_headpos()[2],Body.get_sensor_headpos()[1]};	--b51
 
	local ball = {};
  ball.detect = 0;
	--print("entering arbirary ball detection routine");

  local ballPropsB = ImageProc.connected_ballCandidates(
																													Vision.labelA.dataBall, 
																													Vision.labelA.m, 
																													Vision.labelA.n, 
																													headAngle[2]+10*math.pi/180
																												);

  if (not ballPropsB or #ballPropsB == 0) then return ball; end

	local minEval = math.huge;
	local minId = 0;

	for i = 1, #ballPropsB do
		
		--print("cntr", ballPropsB[i].blCntr, ballPropsB[i].bkCntr, ballPropsB[i].wtCntr, ballPropsB[i].radiusRate)
		--print("bbox", ballPropsB[i].boundingBox[1], ballPropsB[i].boundingBox[2], ballPropsB[i].boundingBox[3], ballPropsB[i].boundingBox[4])

		local check_passed = true; 
		local totalCntr = ballPropsB[i].blCntr + ballPropsB[i].bkCntr + ballPropsB[i].wtCntr;

		-- calculate the probability from those returned value
		if(ballPropsB[i].radiusRate < 0.5 or totalCntr < 50 ) then

			check_passed = false;
		end
	
		if(check_passed) then
			fillRate = totalCntr/((ballPropsB[i].boundingBox[2]-ballPropsB[i].boundingBox[1]+1)*(ballPropsB[i].boundingBox[4]-ballPropsB[i].boundingBox[3]+1));
			if(fillRate < 0.3) then
				check_passed = false;
			end
		end

		if(check_passed) then
			squareRate = (ballPropsB[i].boundingBox[2]-ballPropsB[i].boundingBox[1]+1)/(ballPropsB[i].boundingBox[4]-ballPropsB[i].boundingBox[3]+1);
			if(squareRate > 1) then
				squareRate = 1/squareRate;
			end
			if (squareRate <0.4) then
				check_passed = false;
			elseif (squareRate < 0.65 and headAngle[2] > 55*math.pi/180) then
				check_passed = false;
			end
		end 

		if(check_passed) then
			background_check_offset = 2;
			local checkBox = ballPropsB[i].boundingBox;
			checkBox = getCheckBoxWithOffset (checkBox, background_check_offset);
			--print(unpack(checkBox));
			
			statsResult = ImageProc.bounding_field_stats(Vision.labelA.data, Vision.labelA.m, Vision.labelA.n, colorField, checkBox);
			--print("backgroundRatio", statsResult.backgroundRatio)
			if(statsResult.backgroundRatio < 0.5) then
				check_passed = false;
			end
		end

		if(check_passed) then

			Kcolor = 0.2;
			Kradius = 0.2;
			Kground = 0.2;
			Ksquare = 0.2;
			Kfill = 0.2;
			
			--Kcolor =1.5 * math.sqrt((4/math.pi)*totalCntr) / 90;

			local evaluation = Kcolor * ((ballPropsB[i].blCntr/totalCntr - 0.4)^2 + 
					(ballPropsB[i].wtCntr/totalCntr - 0.4)^2 + 
					(ballPropsB[i].bkCntr/totalCntr - 0.2)^2) + 
					Kradius * (ballPropsB[i].radiusRate - 1)^2 +
					Kground * (statsResult.backgroundRatio -1)^2 +
					Kfill*(fillRate -0.75)^2 +
					Ksquare*(squareRate - 1)^2;

			if(evaluation<minEval) then
				minEval = evaluation;
				minId = i;
			end
		end
	end

	if(minId == 0) then
		--print('exiting the test routine')
	  --os.exit()
		return ball;
	end
	--print("best ball fitted is", minEval, minId)

	ball.propsA = {};
	ball.propsA.centroid = {(ballPropsB[minId].boundingBox[1]+ballPropsB[minId].boundingBox[2])/2, (ballPropsB[minId].boundingBox[3]+ballPropsB[minId].boundingBox[4])/2};
	ball.propsA.area = ballPropsB[minId].wtCntr + ballPropsB[minId].blCntr + ballPropsB[minId].bkCntr;
	ball.propsA.axisMajor = math.max(ballPropsB[minId].boundingBox[2]-ballPropsB[minId].boundingBox[1],ballPropsB[minId].boundingBox[4]-ballPropsB[minId].boundingBox[3]);
	ball.propsA.axisMinor = math.min(ballPropsB[minId].boundingBox[2]-ballPropsB[minId].boundingBox[1],ballPropsB[minId].boundingBox[4]-ballPropsB[minId].boundingBox[3]);
	ball.propsA.boundingBox = ballPropsB[minId].boundingBox;

	--print("propsA: ", ball.propsA.area, ball.propsA.axisMajor, ball.propsA.axisMinor)
	--print("centroid: ", unpack(ball.propsA.centroid))

	-- diameter of the area
	local dArea = math.sqrt((4/math.pi)*ball.propsA.area);
	-- Find the centroid of the ball
	local ballCentroid = ball.propsA.centroid;
	-- Coordinates of ball
	local scale = math.max(dArea/diameter, ball.propsA.axisMajor/diameter);

	--print("interface to previous:",dArea, ballCentroid, scale)

	v = HeadTransform.coordinatesA(ballCentroid, scale);
	v_inf = HeadTransform.coordinatesA(ballCentroid,0.1);

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
  ball.detect = 1;
  ball.r = math.sqrt(ball.v[1]^2 + ball.v[2]^2);
  
  -- How much to update the particle filter
  ball.dr = 0.25*ball.r;
  ball.da = 10*math.pi/180;

	t003 = unix.time();

	--print('exiting the test routine')
	--os.exit()

	return ball;

end

