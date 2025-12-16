import pyrealsense2 as rs
import numpy as np
import cv2
from flask import Flask, Response

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)
colorizer = rs.colorizer()

app = Flask(__name__)

v_tst = 0
def gen_frames():
    while True:
        frames = pipeline.wait_for_frames()
        depth = frames.get_depth_frame()
        if not depth:
            continue
        colorized = colorizer.colorize(depth)
        frame = np.asanyarray(colorized.get_data())
        ret, buffer = cv2.imencode('.jpg', frame)
        if not ret:
            continue
        jpg = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + jpg + b'\r\n')

@app.route('/stream')
def stream():
    return Response(gen_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)

