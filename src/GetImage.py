import requests
import cv2
import numpy as np
from time import sleep
import sys

# The URL of the video stream from the server (replace with the actual IP address of the server)
video_stream_url = 'http://192.168.1.167:5000/video_feed'
response = requests.get(video_stream_url, stream=True)
CONTENT = response.iter_content(chunk_size=1024)
FRAME_DATA = b''

def getFrame():
    global FRAME_DATA
    # Buffer to accumulate the chunks for decoding
    buffer = b''
    
    if response.status_code == 200:
        # print("Successfully connected to the video stream")
        pass
    else:
        print(f"Failed to connect to video stream. Status code: {response.status_code}")
        exit()

    # Iterate over the stream and get individual frames
    for chunk in CONTENT:
        FRAME_DATA += chunk  # Append the new chunk to the buffer
        
        # Look for the boundary separating frames in the stream
        while b'--frame' in FRAME_DATA:
            # Find the start and end of the current frame in the buffer
            start = FRAME_DATA.find(b'\r\n\r\n') + 4  # The start of the frame data
            end = FRAME_DATA.find(b'\xff\xd9\r\n', start)  # The end of the frame
            
            if start == -1 or end == -1:
                break
            
            # Extract the frame data from the buffer
            frame_data = FRAME_DATA[start:(end+2)]

            # Remove the processed data from the buffer
            FRAME_DATA = FRAME_DATA[end + 2:]
            
            return frame_data
            
            

def save_frame_to_file(self, image_data, filename="frame.jpg"):
        """Save the accumulated image data to a file."""
        with open(filename, 'wb') as f:
            f.write(image_data)
        # print(f"Frame saved to {filename}")

while True:
    frame_data = getFrame()
    if (not frame_data):
        continue
    # Convert the byte data to a numpy array
    np_array = np.frombuffer(frame_data, dtype=np.uint8)

    # Decode the image from the numpy array
    frame = cv2.imdecode(np_array, cv2.IMREAD_COLOR)

    if frame is not None:
        # Show the frame in a window (optional)
        # cv2.imshow('Video Frame', frame)
        cv2.imwrite('frame.jpg', frame)
        print("wrote image")
        
    else:
        print("Failed to decode frame")
        print(frame_data)
        sys.exit()
        


# sleep(3)
# # Release the OpenCV window
cv2.destroyAllWindows()
