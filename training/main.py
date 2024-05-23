# python main.py ~/Projects/cchs/training/2024-05-14-10-09-15 ~/Projects/cchs/training/2024-05-14-10-15-32 ~/Projects/cchs/training/2024-05-19-10-16-09 ~/Projects/cchs/training/2024-05-19-10-41-25 ~/Projects/cchs/training/2024-05-19-12-37-04
# toco --output_file=self_drive_model.tflite --keras_model_file=self_drive_model.h5
# pip install -U tf_keras
# python3.11 not 12
# pip install tensorflow==2.15.1  
# pip install keras==3.2.1


# OS contains functions such as reading files
import os
# for parsing file
import struct
# for generating filenames from sensor data
from datetime import datetime, timezone
import pytz
# numpy is a popular library used for operations on numeric arrays
import numpy as np
# Python Image Library for image processing
from PIL import Image
# We will use TensorFlow and Keras for building the neural network
import tensorflow as tf

Sequential = tf.keras.models.Sequential
Input = tf.keras.layers.Input
Conv2D = tf.keras.layers.Conv2D
MaxPooling2D = tf.keras.layers.MaxPooling2D
Flatten = tf.keras.layers.Flatten
Dense = tf.keras.layers.Dense
Dropout = tf.keras.layers.Dropout
EarlyStopping = tf.keras.callbacks.EarlyStopping

import matplotlib.pyplot as plt

# sklearn allows us to easily split the dataset
from sklearn.model_selection import train_test_split

# os.environ["TF_USE_LEGACY_KERAS"] = "1" 

def get_time_stamp_from_ms(ms):
    seconds = ms / 1000.0
    utc_dt = datetime.fromtimestamp(seconds, tz=timezone.utc)
    local_tz = os.getenv('TZ')
    if not local_tz:
        local_tz = pytz.timezone('Australia/Melbourne')
    local_dt = utc_dt.astimezone(local_tz)
    return local_dt.strftime('%Y-%m-%d-%H:%M:%S.%f')[:-3]

def read_sensor_data(input_dir):
    sensor_file_path = os.path.join(input_dir, 'sensor_data.bin')
    sensor_struct_format = 'qbb3H6f'
    data = []

    with open(sensor_file_path, 'rb') as file:
        while True:
            binary_data = file.read(struct.calcsize(sensor_struct_format))
            if not binary_data:
                break
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
            data.append(sen_prog_data)

    return data

def preprocess_images(input_dir, sensor_data):
    data = []
    labels = []

    for entry in sensor_data:
        timestamp = entry['timeMs']
        throttle = entry['throttle']
        steering = entry['steering']

        timestamp_str = get_time_stamp_from_ms(timestamp)
        image_path = os.path.join(input_dir, f"source_image_{timestamp_str}.jpg")

        if (os.path.exists(image_path)):
            print(f"Preprocessing {image_path}. throttle={throttle}, steering={steering}")
            image = Image.open(image_path).convert('L') # open jpg and convert to grayscale
            image = image.resize((160, 120))
            image_array = np.array(image)
            data.append(image_array)
            labels.append([throttle, steering])

    data = np.array(data)
    labels = np.array(labels)

    # Normalize the data
    data = data / 255.0

    # Reshape data for the CNN input
    data = data.reshape(data.shape[0], 120, 160, 1)

    return data, labels


def main(input_dirs):
    all_data = []
    all_labels = []

    for input_dir in input_dirs:
        sensor_data = read_sensor_data(input_dir)
        data, labels = preprocess_images(input_dir, sensor_data)
        all_data.append(data)
        all_labels.append(labels)

    # combile multiple arrays into single arrays
    all_data = np.concatenate(all_data, axis=0)
    all_labels = np.concatenate(all_labels, axis=0)

    # split the data into training and testing sets
    x_train, x_test, y_train, y_test = train_test_split(all_data, all_labels, test_size=0.2, random_state = 42)

    # define the cnn model
    model = Sequential([
        Input(shape=(120, 160, 1)),
        Conv2D(32, (3, 3), activation='relu'),
        MaxPooling2D((2, 2)), 
        Conv2D(64, (3, 3), activation='relu'),
        MaxPooling2D((2, 2)),
        Conv2D(128, (3, 3), activation='relu'),
        MaxPooling2D((2, 2)),
        Conv2D(256, (3, 3), activation='relu'),
        MaxPooling2D((2, 2)),
        Flatten(),
        Dense(512, activation='relu'),
        Dropout(0.5),
        Dense(2) # output for throttle and steering
    ])

    # compile the model
    model.compile(optimizer='adam', loss='mean_squared_error')

    model.summary()
    # define earlystopping callback
    early_stopping = EarlyStopping(monitor='val_loss', patience=5, restore_best_weights=True)
    
    # train the model
    history = model.fit(x_train, y_train, epochs=50, validation_data=(x_test, y_test), callbacks=[early_stopping])

    # # save the model
    model.save('self_drive_model.h5')

    print("Model traning done")

    # convert the model to tensorflow lite format
    print("Converting...")
    converter = tf.lite.TFLiteConverter.from_keras_model(model)
    tflite_model = converter.convert()

    # Save the converted model.
    with open('model.tflite', 'wb') as f:
        f.write(tflite_model)
    
    print("Conversion complete")

    test_loss = model.evaluate(x_test, y_test)
    print(f'Test loss: {test_loss}')

    # plot traning and valuation loss values
    plt.plot(history.history['loss'], label='Training Loss')
    plt.plot(history.history['val_loss'], label='Validation Loss')
    plt.title('Model Loss')
    plt.xlabel('Epoch')
    plt.ylabel('Loss')
    plt.legend(loc='upper right')
    plt.show()

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Train self-driving model')
    parser.add_argument('input_dir', type=str, nargs='+', help="Folder containing sensor data file and images")
    args = parser.parse_args()

    main(args.input_dir)