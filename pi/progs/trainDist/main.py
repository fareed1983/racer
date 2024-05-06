import os
import struct
from datetime import datetime,timezone
import pytz
import cv2
import threading
import time
from picamera2.picamera2 import *

# Follow this guide for picamera2 installation https://forums.raspberrypi.com/viewtopic.php?t=361758

# pip install opencv-python



def get_time_stamp_from_ms(ms):
    seconds = ms / 1000.0
    utc_dt = datetime.fromtimestamp(seconds, tz=timezone.utc)
    local_tz = pytz.timezone(os.getenv('TZ'))
    if not local_tz:
        local_tz = timezone.utc
    local_dt = utc_dt.astimezone(local_tz)
    return local_dt.strftime('%Y-%m-%d-%H:%M:%S.%f')[:-3]
    

def read_sensor_data(output_folder):
    sensor_fifo_path = '/tmp/sensor_data_fifo'
    sensor_struct_format = 'qbb3H6f'

    if not os.path.exists(sensor_fifo_path):
        os.mkfifo(fifo.path)

    sensor_file_path = os.path.join(output_folder, 'sensor_data.bin')

    try:
        with open(sensor_file_path, 'ab') as out_file:
            with open(sensor_fifo_path, 'rb') as fifo:
                print("Opened FIFO for reading")
                while True:
                    binary_data = fifo.read(struct.calcsize(sensor_struct_format))
                    if not binary_data:
                        break;

                    upd = struct.unpack(sensor_struct_format, binary_data)
                    sen_prog_data = {
                        'timeMs': upd[0],
                        'throttle': upd[1],
                        'steering': upd[2],
                        'dists': upd[3:6],
                        'accX': upd[6],
                        'accY': upd[7],
                        'accZ': upd[8],
                        'gyroX': upd[9],
                        'gyroY': upd[10],
                        'gyroZ': upd[11]
                    }
                    sen_prog_data['timeMs']=get_time_stamp_from_ms(sen_prog_data['timeMs'])
                    #print(sen_prog_data)
                    out_file.write(binary_data)

    except KeyboardInterrupt:
        print("Ctrl + C detected. Exiting...")
    finally:
        fifo.close()
        out_file.flush()
        os.fsync(out_file.fileno())
        out_file.close()

def capture_images(output_folder):


    picam2 = Picamera2()
    config = picam2.create_still_configuration(main= {"size": (800, 600)}, lores = {"size": (800, 600)}, display = None, buffer_count = 3, queue = False)
    picam2.configure(config)

    picam2.start(show_preview=False)

    try:
        for i in range(1000):
            img = picam2.capture_array("lores")            
            edges = cv2.Canny(img, 50, 150, apertureSize=3)
            dialated_edges = cv2.dilate(edges, np.ones((3,3), np.uint8), iterations = 1)

            processed_image_path = os.path.join(output_folder, f'processed_image_{i}.jpg')
            cv2.imwrite(processed_image_path, dialated_edges)

            cv2.imshow("Processed Image", cv2.resize(dialated_edges, (0,0), fx=0.5, fy=0.5))
            if cv2.waitKey(1) & 0xff == ord('q'):
                break
    finally:
        picam2.stop()
        cv2.destroyAllWindows()

def main():
    try:
        # Create a folder with the current timestamp to store data of current run
        output_folder = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
        if not os.path.exists(output_folder):
            os.makedirs(output_folder)

        # Start sensor data reading thread
        sensor_thread = threading.Thread(target=read_sensor_data, args=(output_folder,))
        sensor_thread.start()

        # Start main program for webcam capturing and processing
        capture_images(output_folder)

    except KeyboardInterrupt:
        print("Ctrl+C detected. Exiting...")
    


if __name__ == "__main__":
    main()