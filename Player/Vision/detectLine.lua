module(..., package.seeall);

require('Config');	-- For Ball and Goal Size
require('ImageProc');
require('HeadTransform');	-- For Projection
require('Vision');

-- Dependency
require('Detection');

-- Define Color
colorOrange = 1;
colorYellow = 2;
colorCyan = 4;
colorField = 8;
colorWhite = 16;

min_white_pixel = Config.vision.line.min_white_pixel or 200;
min_green_pixel = Config.vision.line.min_green_pixel or 5000;

--min_width=Config.vision.line.min_width or 4;
max_width=Config.vision.line.max_width or 8;
connect_th=Config.vision.line.connect_th or 1.4;
max_gap=Config.vision.line.max_gap or 1;
min_length=Config.vision.line.min_length or 3;
lwratio = Config.vision.line.lwratio or 2

function in_bbox(pointx, pointy, bbox)
  if (pointx>=bbox[1] and pointx<=bbox[2]
   	and pointy>=bbox[3] and pointy<=bbox[4]) then
   	return true
  else 
    return false 
  end
end

function detect()
  --TODO: test line detection
  line = {};
  line.detect = 0;

  linePropsB = ImageProc.field_lines(Vision.labelB.data, Vision.labelB.m,
		 Vision.labelB.n, max_width, connect_th, max_gap, min_length);

  if #linePropsB==0 then 
    --print('linePropsB nil')
    return line; 
  end

  line.propsB=linePropsB;
	nLines=math.min(#line.propsB,12);

  vcm.add_debug_message(string.format(
    "Total %d lines detected\n" ,nLines));

  line.detect = 1;
  line.v={};
  line.endpoint={};
  line.angle={};
  line.meanpoint={};
  line.length={}

  bestindex=1;
  bestlength=0;
  linecount=0;

  for i = 1, nLines do
    line.endpoint[i] = vector.zeros(4);
    line.v[i]={};
    line.v[i][1]=vector.zeros(4);
    line.v[i][2]=vector.zeros(4);
    line.angle[i] = 0;
  end

  for i=1, nLines do
    local valid = true;

    if line.propsB[i].endpoint[3] < HeadTransform.get_horizonB() or line.propsB[i].endpoint[4] < HeadTransform.get_horizonB() then
      valid = false;
    end

    if vcm.get_spot_detect() == 1 then
    	local spotbboxB = vcm.get_spot_bboxB()
    	if in_bbox(line.propsB[i].endpoint[1], line.propsB[i].endpoint[3], spotbboxB) or
        in_bbox(line.propsB[i].endpoint[2], line.propsB[i].endpoint[4], spotbboxB) then
        valid = false
      end
    end

    if valid then
      local ratio = line.propsB[i].length/line.propsB[i].max_width;
      if ratio<=lwratio then 
      	valid = false
      end
    end 

    if valid then
      local vendpoint = {};
    	vendpoint[1] = HeadTransform.coordinatesB(vector.new(
  	    {line.propsB[i].endpoint[1], line.propsB[i].endpoint[3]}),1);
    	vendpoint[2] = HeadTransform.coordinatesB(vector.new(
  		  {line.propsB[i].endpoint[2],line.propsB[i].endpoint[4]}),1);
      linecount=linecount+1;
    	line.length[linecount]=length;
    	line.endpoint[linecount]= line.propsB[i].endpoint;
    	vendpoint[1] = HeadTransform.projectGround(vendpoint[1],0);
    	vendpoint[2] = HeadTransform.projectGround(vendpoint[2],0);
    	line.v[linecount]={};
    	line.v[linecount][1]=vendpoint[1];
    	line.v[linecount][2]=vendpoint[2];
    	local angle = math.atan2(vendpoint[1][2]-vendpoint[2][2],
      vendpoint[1][1]-vendpoint[2][1]);
      if angle<=0 then angle=angle+math.pi end
      line.angle[linecount] = angle;
      line.meanpoint[linecount]={};
      line.meanpoint[linecount][1]=line.propsB[i].meanpoint[1];
      line.meanpoint[linecount][2]=line.propsB[i].meanpoint[2];
    end

  end

  nLines = linecount;
  --TODO

  return line;
end
