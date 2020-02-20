import base64
import os
import glob

def SendRawImages(imglist):
    os.chdir("/home/pi/RPi_v2/correct_images")

    for f_name in glob.glob('*.jpg'):
        with open(f_name, "rb") as img_file:
            encoded_string = base64.b64encode(img_file.read())
            imglist.append(str(encoded_string))
    # print(correctImages[0])

    os.chdir("/home/pi/RPi_v2")
    # with open('image_log_array.txt', 'w') as f:
    #     f.write(str(correctImages))

    return imglist
