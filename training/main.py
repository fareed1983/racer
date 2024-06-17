# OS contains functions such as reading files
import os
# os.environ["TF_USE_LEGACY_KERAS"] = "1"

import random

# for parsing file
import struct
# for generating filenames from sensor data
from datetime import datetime, timezone
import pytz
# numpy is a popular library used for operations on numeric arrays
import numpy as np
# Python Image Library for image processing
from PIL import Image, ImageEnhance, ImageFilter
# We will use TensorFlow and Keras for building the neural network
import tensorflow as tf
import keras

Sequential = tf.keras.models.Sequential
Input = tf.keras.layers.Input
Conv2D = tf.keras.layers.Conv2D
MaxPooling2D = tf.keras.layers.MaxPooling2D
Flatten = tf.keras.layers.Flatten
Dense = tf.keras.layers.Dense
Dropout = tf.keras.layers.Dropout
BatchNormalization = tf.keras.layers.BatchNormalization

l2 = tf.keras.regularizers.l2
Adam = tf.keras.optimizers.Adam
EarlyStopping = tf.keras.callbacks.EarlyStopping

DIM = (160, 120) # (width, height)
# DIM = (192, 144) # (width, height)
EPOCHS = 60

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




def gen_data_lists(input_dir, sensor_data):
    paths = []
    labels = []

    for entry in sensor_data:
        timestamp = entry['timeMs']
        throttle = entry['throttle']
        steering = entry['steering']

        timestamp_str = get_time_stamp_from_ms(timestamp)
        idx = 1
        while True:
            image_path = os.path.join(input_dir, f"source_image_{timestamp_str}_{idx:03d}.jpg")
            if not os.path.exists(image_path):
                break

            paths.append(image_path)
            labels.append([throttle, steering])

            print(f"Adding {image_path}. throttle={throttle}, steering={steering}")

            idx += 1

    labels = np.array(labels)

    # Normalize the data
    labels = labels / 100.0

    return paths, labels

class CustomDataGen(tf.keras.utils.Sequence):
    def __init__(self, data, dim = DIM, batch_size = 32, shuffle = True, augmentations=None, **kwargs):
        super().__init__(**kwargs)
        self.data = data
        self.dim = dim
        self.batch_size = batch_size
        self.indexes = np.arange(len(self.data))
        self.shuffle = shuffle
        self.augmentations = augmentations or []

        self.on_epoch_end()

    def __len__(self):
        num_augmentations = len(self.augmentations) + 1
        return int(np.ceil(len(self.data) * num_augmentations / self.batch_size))
    
    def on_epoch_end(self):
        if (self.shuffle):
            np.random.shuffle(self.indexes)

    def __getitem__(self, index):
        X = []
        y = []
        
        num_augmentations = len(self.augmentations) + 1 

        # the index param in this funciton will assume augmentations are included
        start_index = index * self.batch_size // num_augmentations
        end_index = (index + 1) * self.batch_size // num_augmentations


        batch_indexes = self.indexes[start_index:end_index]
        
        batch_data = [self.data[k] for k in batch_indexes]
        for path, label in batch_data:
            # Load and preprocess the image
            # print(f"Loading {path}. throttle={label[0]}, steering={label[1]}")
            image = Image.open(path).convert('L')  # open jpg and convert to grayscale
            image = image.resize(self.dim)
            image_array = np.array(image)
            image_array = image_array / 255.0  # Normalize the image

            # Append original image data and label
            X.append(image_array)
            y.append(label)

            for augmentation in self.augmentations:
                aug_image = image.copy()
                if augmentation == 'flip':
                    aug_image = aug_image.transpose(Image.FLIP_LEFT_RIGHT);
                    aug_label = [label[0], -label[1]]
                elif augmentation == 'brightness':
                    factor = random.uniform(0.5, 1.5)
                    enhancer = ImageEnhance.Brightness(aug_image)
                    aug_image = enhancer.enhance(factor)
                    aug_label = label
                elif augmentation == 'contrast':
                    factor = random.uniform(0.5, 1.5)
                    enhancer = ImageEnhance.Contrast(aug_image)
                    aug_label = label
                elif augmentation == 'blur':
                    radius = random.uniform(0.5, 2.0)
                    aug_image = aug_image.filter(ImageFilter.GaussianBlur(radius))
                    aug_label = label
            
                aug_image_array = np.array(aug_image)
                aug_image_array = aug_image_array / 255.0

                # Append augmented image data and modified label
                X.append(aug_image_array)
                y.append(aug_label)

        X = np.array(X)
        y = np.array(y)

        # Reshape data for the CNN input
        X = X.reshape(X.shape[0], self.dim[1], self.dim[0], 1) 

        return X, y
    

def main(input_dirs, augmentations):

    augmentations = augmentations or []
    
    all_paths = []
    all_labels = []

    for input_dir in input_dirs:
        sensor_data = read_sensor_data(input_dir)
        paths, labels = gen_data_lists(input_dir, sensor_data)
        all_paths.append(paths)
        all_labels.append(labels)

    # combile multiple arrays into single arrays
    all_paths = np.concatenate(all_paths, axis=0)
    all_labels = np.concatenate(all_labels, axis=0)

    all_data = list(zip(all_paths, all_labels))

    # split the data into training and testing sets
    train_data, val_data = train_test_split(all_data, test_size=0.2, random_state=42)
    
    print(f"Will use {len(train_data)} images for training ({len(train_data) * (len(augmentations) + 1)} with augmentations) & {len(val_data)} images for validation")
    # define the cnn model

    model = Sequential([
        Input(shape=(DIM[1], DIM[0], 1)),
        
        Conv2D(24, (5, 5), strides=(2, 2), activation='relu'),
        Conv2D(24, (5, 5), strides=(2, 2), activation='relu'),
        Conv2D(64, (5, 5), activation='relu'),
        MaxPooling2D(pool_size=(2, 2)),
        Conv2D(64, (3, 3), activation='relu'),
        MaxPooling2D(pool_size=(2, 2)),
        Conv2D(64, (3, 3), activation='relu'),

        Flatten(),
      
        Dense(100, activation='relu'),
        Dropout(0.4),
        Dense(50, activation='relu'),
        Dropout(0.3),
        Dense(10, activation='relu'),
        Dense(2)  # Output for throttle and steering
    ])


    # model = Sequential([
    #     Input(shape=(DIM[1], DIM[0], 1)),
        
    #     Conv2D(24, (5, 5), strides=(2, 2), activation='relu'),
    #     Conv2D(32, (5, 5), strides=(2, 2), activation='relu'),
    #     Conv2D(64, (5, 5), strides=(2, 2), activation='relu'),
    #     Conv2D(64, (3, 3), activation='relu'),
    #     Conv2D(64, (3, 3), activation='relu'),
        
    #     Flatten(),
        
    #     Dense(100, activation='relu'),
    #     Dropout(0.5),
    #     Dense(50, activation='relu'),
    #     Dropout(0.5),
    #     Dense(10, activation='relu'),
    #     Dense(2)  # Output for throttle and steering
    # ])

    # compile the model
    model.compile(optimizer='adam', loss='mse')

    model.summary()
    # define earlystopping callback
    early_stopping = EarlyStopping(monitor='val_loss', patience=5, restore_best_weights=True)
    
    train_gen = CustomDataGen(train_data, batch_size=32, dim=DIM, shuffle=True, augmentations=augmentations)
    val_gen = CustomDataGen(val_data, batch_size=32, dim=DIM, shuffle=False)

    # train the model
    history = model.fit(train_gen, epochs=EPOCHS, validation_data=val_gen, callbacks=[early_stopping])

    # # save the model
    model.export('self_drive_model')

    print("Model traning done")

    test_loss = model.evaluate(val_gen)
    print(f'Test loss: {test_loss}')

    # convert the model to tensorflow lite format
    print("Converting...")
    model = keras.layers.TFSMLayer("self_drive_model", call_endpoint='serve') # TensorFlow >= 2.16.0
    converter = tf.lite.TFLiteConverter.from_keras_model(model)

    converter.optimizations = [tf.lite.Optimize.DEFAULT]
    tflite_model = converter.convert()
    tf.lite.experimental.Analyzer.analyze(model_content=tflite_model)
    open("self_drive_model.tflite", "wb").write(tflite_model)

    print("Conversion complete")


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

    parser = argparse.ArgumentParser(description='Self-driving model trainer')
    # input arguments for the type of augmentations to perform
    parser.add_argument('-a', '--augmentations', type=str, nargs='+', required=False, help="Augmentations to use when training. Options: flip, brightness, contrast, blur.")
    parser.add_argument('-i', '--input-dirs', type=str, nargs='+', help="Folder containing sensor data file and images")
    args = parser.parse_args()

    main(args.input_dirs, args.augmentations)