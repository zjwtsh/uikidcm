require('Config');

Camera = require(Config.dev.camera);

if Config.vision.use_arbitrary_ball then
  Camera.init(1);  --for 640x480
else
  Camera.init(0);  -- 1280x720
end
