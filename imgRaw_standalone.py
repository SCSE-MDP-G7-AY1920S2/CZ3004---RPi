import base64
import os
import glob
import json
import math

os.chdir("/home/pi/RPi_v2/correct_images")

# commsList[ANDROID].write(';{"com":"statusUpdate", "status":"Start sending raw images..."}')
# ACK

for f_name in glob.glob('*.jpg'):
    with open(f_name, "rb") as img_file:
        start_index = 0
        count = 0

        encoded_string = base64.b64encode(img_file.read())
        string_length = len(encoded_string)
        iter = math.ceil(string_length/700)
        print("String length in bytes: " + string_length/1024)
        print("Number of iterations: " + iter)

        for i in iter:
            count += 1
            end_index = start_index + 700

            substring = encoded_string[start_index:end_index]
            data = {'com': 'Raw Image String','count': count, 'imgRaw': substring}
            print(json.dumps(data))
            # commsList[ANDROID].write(json.dumps(data))
            start_index += 700

# commsList[ANDROID].write(';{"com":"statusUpdate", "status":"Finish sending raw images..."}')
# ACK
os.chdir("/home/pi/RPi_v2")

