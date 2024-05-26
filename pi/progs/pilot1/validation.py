# Load model, start a thread for image capture and resize
# and use model to predict steering and throttle
import cv2
import numpy as np
import argparse
import tensorflow as tf
from PIL import Image
import cv2
import os
import time

inference_times = []

def drive_frame(image, interpreter, input_details, output_details):
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
            print(output_data)
            throttle, steering = output_data[0]

            throttle = throttle * 100
            steering = steering * 100

            print(f"t:{throttle}, s: {steering}")

            throttle = np.clip(throttle, 0, 50)
            steering = np.clip(steering, -100, 100)

            print(f"T:{throttle}, S: {steering}")

            throttle = int(throttle)
            steering = int(steering)

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


def map_value(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

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
            drive_frame(image, interpreter, input_details, output_details)
    except KeyboardInterrupt:
        cv2.destroyAllWindows()

else:
        print("No folder")


