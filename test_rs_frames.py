import pyrealsense2 as rs
import time

pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

print("starting pipeline...")
profile = pipeline.start(config)
print("pipeline started")

for i in range(30):
    frames = pipeline.poll_for_frames()
    if not frames:
        print(f"[{i}] no frames")
    else:
        color = frames.get_color_frame()
        depth = frames.get_depth_frame()
        print(f"[{i}] color={bool(color)} depth={bool(depth)}")
    time.sleep(0.1)

pipeline.stop()
print("done")
