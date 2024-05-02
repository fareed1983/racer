import os
import struct
from datetime import datetime,timezone
import pytz

def get_time_stamp_from_ms(ms):
    seconds = ms / 1000.0
    print(seconds)
    utc_dt = datetime.fromtimestamp(seconds, tz=timezone.utc)
    local_tz = pytz.timezone(os.getenv('TZ'))
    if not local_tz:
        local_tz = timezone.utc
    local_dt = utc_dt.astimezone(local_tz)
    return local_dt.strftime('%Y-%m-%d-%H:%M:%S.%f')[:-3]
    

sensor_fifo_path = '/tmp/sensor_data_fifo'
sensor_struct_format = 'qbb3H6f'

if not os.path.exists(sensor_fifo_path):
    os.mkfifo(fifo.path)

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
        print(sen_prog_data)