import os
import struct
from datetime import datetime,timezone
import pytz
import cv2
import threading
import time
from picamera2.picamera2 import *




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
    config = picam2.create_still_configuration(main= {"size": (4056, 3040)}, lores = {"size": (480, 320)}, display = "lores", buffer_count = 3, queue = False)
    picam2.configure(config)

    #picam2.set_controls({"ExposureTime": 10000, "AnalogueGain": 5}) #Shutter time and analogue signal boost
    picam2.start(show_preview=True)

    time.sleep(10)  #enjoy the preview

    t_0 = time.monotonic()
    img = picam2.capture_array("lores") #this takes a picture. img can be used with cv2
    t_1 = time.monotonic()
    picam2.close() #when you're done taking photos, this closes the camera connection

    print("taking the photo took %s seconds", round(t_1-t_0, 3))
    print("width height\t:", *img.shape[0:2][::-1])

    cv2.imshow("Title", cv2.resize(img, (0,0), fx=0.25, fy=0.25)) #resize the image to a quarter of the original size
    cv2.waitKey(0) #Wait until a key is pressed while the img window is selected

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