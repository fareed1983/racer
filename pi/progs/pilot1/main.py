# Load model, start a thread for image capture and resize
# and use model to predict steering and throttle
import cv2
import numpy as np
import argparse
import tensorflow as tf
from PIL import Image
from picamera2.picamera2 import *
from RPLCD.i2c import CharLCD
import cv2
from pwm import Adafruit_PWMServoDriver

inference_times = []

def drive_frame(image, interpreter, input_details, output_details, pwm):
            gray_image = image.resize((160, 120))
            image_array = np.array(gray_image, dtype=np.float32)
            
            image_array /= 255.0
            
            # Expand dimensions to match model input
            input_data = np.expand_dims(image_array, axis=0)  # Add batch dimension
            input_data = np.expand_dims(input_data, axis=-1)  # Add channel dimension

            # Set the input tensor
            interpreter.set_tensor(input_details[0]['index'], input_data)

            start_time = time.time()
            interpreter.invoke()
            end_time = time.time()
            inference_time = (end_time - start_time) * 1000  # Convert to milliseconds
            inference_times.append(inference_time)
            if len(inference_times) > 10:
                inference_times.pop(0)


            # Get the output tensor
            output_data = interpreter.get_tensor(output_details[0]['index'])
            throttle, steering = output_data[0]
            throttle = int(throttle * 100.0)
            steering = int(steering * 100.0)

            throttle = np.clip(throttle, 20, 50)
            steering = np.clip(steering, -100, 100)

            print(f"T:{throttle}, S: {steering}")

            display_image = cv2.cvtColor(np.array(image), cv2.COLOR_GRAY2BGR)
            cv2.putText(display_image, f"T:{throttle} S:{steering}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
            # Draw throttle bar
            cv2.rectangle(display_image, (50, 205), (80, 200), (255, 255, 255), -1)
            cv2.rectangle(display_image, (50, 200), (80, 200 - throttle * 2), (0, 255, 0), -1)
            
            # Draw steering bar
            cv2.rectangle(display_image, (245, 50), (255, 80), (255, 255, 255), -1)
            cv2.rectangle(display_image, (250, 50), (250 + steering * -1, 80), (0, 0, 255), -1)
            

            # Display the last 10 inference times
            for i, inf_time in enumerate(inference_times):
                text = f"Time {i+1}: {inf_time:.2f} ms"
                cv2.putText(display_image, text, (10, display_image.shape[0] - 10 - i * 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 25, 0), 1, cv2.LINE_AA)

            # Display the image
            cv2.imshow("Processed Image", display_image)
            cv2.waitKey(1)

            t = map_value(throttle, -100, 100, ESC_MIN, ESC_MAX) 
            pwm.writeMicroseconds(ESC_IDX, t)

            s = map_value(steering, -100, 100, SER_MIN, SER_MAX) 
            pwm.writeMicroseconds(SER_IDX, s)

            # try:
            #     lcd.clear()
            #     lcd.write_string(f"T:{int(throttle)}  S:{int(steering)}  ")
            # except:
            #     print("Err writing to lcd")
            #     lcd = CharLCD(i2c_expander='PCF8574', address=0x27, port=1, cols=16, rows=2, dotsize=8)

def map_value(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


SER_IDX = 15
ESC_IDX = 14

SER_MIN = 1150
SER_MID = 1500
SER_MAX = 1850

ESC_MIN = 1200
ESC_MID = 1500
ESC_MAX = 1800

def main():

    pwm = Adafruit_PWMServoDriver()
    pwm.begin()
    pwm.setOscillatorFrequency(27000000)
    pwm.setPWMFreq(50)
    pwm.writeMicroseconds(SER_IDX, SER_MID)
    pwm.writeMicroseconds(ESC_IDX, ESC_MID)
    
    # lcd = CharLCD(i2c_expander='PCF8574', address=0x27, port=1, cols=16, rows=2, dotsize=8)

    parser = argparse.ArgumentParser(description = 'RC Car Pilot v1.0')
    parser.add_argument('-m', '--model-path', type=str, required=True, help="TensorFlow lite model file to use")
    parser.add_argument('-f', '--folder', type=str, required=False, help="Folder where image files are located")
    args = parser.parse_args()

    interpreter = tf.lite.Interpreter(model_path = args.model_path)
    interpreter.allocate_tensors()

    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()


    if args.folder:
        print("Opening folder", args.folder)
        image_files = sorted([f for f in os.listdir(args.folder) if f.endswith('.jpg')])
        try:
            for image_file in image_files:
                image_path = os.path.join(args.folder, image_file)
                image = Image.open(image_path).convert('L')
                drive_frame(image, interpreter, input_details, output_details, pwm)
        except KeyboardInterrupt:
            cv2.destroyAllWindows()

    else:
        picam2 = Picamera2()

        config = picam2.create_still_configuration(main = {"size": (640, 480)}, lores = {"size": (640, 480)}, display = None, buffer_count = 3, queue = False)
        picam2.configure(config)
        picam2.start(show_preview=False)

        try:
            while True:
                yuv = picam2.capture_array("lores")
                rgb = cv2.cvtColor(yuv, cv2.COLOR_YUV420p2RGB)
                image = Image.fromarray(rgb).convert('L')
                drive_frame(image, interpreter, input_details, output_details, pwm)
        
        except KeyboardInterrupt:
            picam2.stop()
            cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
