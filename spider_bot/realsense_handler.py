
import pyrealsense2 as rs
import time
import numpy as np
import cv2
import pyfakewebcam

camera = pyfakewebcam.FakeWebcam('/dev/video30', 640, 960)

### sometimes the realsense has to restart because it showed Frame didn't arrive
### this is likely plug-unplug USB
### https://github.com/IntelRealSense/librealsense/issues/6628#issuecomment-646558144
ctx = rs.context()
devices = ctx.query_devices()
for dev in devices:
	dev.hardware_reset()

### Start the pipeline after that
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

print("start realsense")
try:
	while True:
		frames = pipeline.wait_for_frames()
		color_frame = frames.get_color_frame()
		depth_frame = frames.get_depth_frame()
		
		depth_image = np.asanyarray(depth_frame.get_data())
		color_image = np.asanyarray(color_frame.get_data())

		# depth_image_3d = np.dstack((depth_image,depth_image,self.cv_image))
		depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

		images = np.vstack((color_image, depth_colormap))
		display_image = cv2.cvtColor(images, cv2.COLOR_BGR2RGB)
		# cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
		# cv2.imshow('color', color_image)
		# cv2.imshow('depth', depth_colormap)
		# cv2.waitKey(1)

		camera.schedule_frame(display_image)
	
	
	#print(len(depth_image))
	#time.sleep(0.5)
	
finally:

    # Stop streaming
    pipeline.stop()		
