from flask import Flask, render_template, Response
from picamera2 import Picamera2
from threading import Thread
import time
import io
import piexif
from PIL import Image as PILImage
from PIL.JpegImagePlugin import JpegImageFile

app = Flask(__name__, template_folder="Gemini/templates")
camera = Picamera2()

config = camera.create_video_configuration(
    main={"size": (640, 480)},
    controls={"FrameDurationLimits": (15000, 15000)},  # About 60fps
    buffer_count=2
)
camera.configure(config)
camera.start()
time.sleep(1)  # Allow time for the camera to start

global startTime
startTime = time.time()
global streamFps  # Target FPS in milliseconds, 30fps is ~33 milliseconds
streamFps = 33

def millis():  # Returns time in milliseconds since program start
    return round((time.time() - startTime) * 1000)

def embed_timestamp_in_exif(image_data, timestamp):
    # Open the image with Pillow
    img = PILImage.open(io.BytesIO(image_data))

    # Convert the timestamp to a string format (e.g., 'YYYY:MM:DD HH:MM:SS')
    # timestamp_str = time.strftime('%Y:%m:%d %H:%M:%S', time.gmtime(timestamp))
    current_time = time.time()
    seconds = int(current_time)
    nanoseconds = int((current_time - seconds) * 1e9)
    timestamp_str = F"{seconds}:{nanoseconds}"

    # Get the current EXIF data (if exists) and modify it
    exif_dict = piexif.load(img.info.get("exif", b""))

    # Add the timestamp to the EXIF DateTimeOriginal tag (0x9003)
    exif_dict["0th"][piexif.ImageIFD.DateTime] = timestamp_str

    # Convert the EXIF dictionary back to binary format
    exif_bytes = piexif.dump(exif_dict)

    # Create a new BytesIO object to save the image with updated EXIF metadata
    new_stream = io.BytesIO()
    img.save(new_stream, format='JPEG', exif=exif_bytes)

    # Reset the stream to the beginning to allow reading
    new_stream.seek(0)

    return new_stream.read()  # Return the image with the updated EXIF metadata

def cameraStream():  # Captures frames from the camera and writes them to a buffer
    global frame
    global streamFps
    lastFrameTime = 0
    while True:
        if ((millis() - lastFrameTime) >= streamFps):
            stream = io.BytesIO()
            timestamp = time.time()  # Capture the timestamp of the frame
            
            camera.capture_file(stream, format='jpeg')  # Capture image in JPEG format
            stream.seek(0)  # Go to the start of the buffer
            frame_data = stream.read()
            
            # Embed the timestamp in EXIF metadata
            frame_data_with_timestamp = embed_timestamp_in_exif(frame_data, timestamp)
            
            frame = (frame_data, timestamp)  # Store both the frame data and timestamp
            lastFrameTime = millis()
            stream.truncate()  # Clean up buffer memory

def videoStream():  # Returns the current (new) frame plus some metadata
    global frame
    lastFrame = None
    while True:
        if frame != lastFrame:
            lastFrame = frame
            frame_data, timestamp = frame  # Unpack frame and timestamp
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_data + b'\r\n')

@app.route('/')
def index():
    return '''
    <html>
        <head>
            <title>Raspberry Pi Camera Stream</title>
        </head>
        <body>
            <h1>Live Camera Stream</h1>
            <img src="/video_feed">
        </body>
    </html>
    '''

@app.route('/video_feed')
def video_feed():  # Returns the current frame plus metadata
    return Response(videoStream(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    streamThread = Thread(target=cameraStream)
    streamThread.start()  # Runs in background so non-blocking
    app.run(host='0.0.0.0', debug=False)
