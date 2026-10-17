# take in a png image, turn it into a image.h file
# usage: ./image_encoder.py img_name.png out_file_name 
#
# format of image.h file:
#     54400 length array of 16bit pixel values
#     pixels are drawn in row major order
#
#     use const unsigned short for 16bit values
#     pixel values are in RRRRR_GGGGGG_BBBBB (5, 6, 5)

from PIL import Image
import numpy as np
import sys

image_input_max_num = 256 # 8 bits per color in a normal image
image_output_max_num_rgb = [2**5, 2**6, 2**5] # 5 bits for red and blue, 6 for green

byte_swapped = True

if (len(sys.argv) != 3):
    print("you need to pass in an image and a name")
    exit()

img = Image.open(sys.argv[1])
file_name = sys.argv[2]
array = np.asarray(img, dtype=np.uint64)

pixel_strings_to_add = []

# convert array from uint8 to 5, 6, 5 
for r in range(array.shape[0]):
    for c in range(array.shape[1]):


        rgba = array[r, c]
        pixel_string = ""

        # edge case, if the pixel is transparent at all, then set it to pink instead
        if(rgba[3] != 255):
            pixel_string = "1111100000011111"
        else:
            # for each value, multiply by 32 (64 if green) then divide by 256
            eight_bpp_rgb = [rgba[0] * image_output_max_num_rgb[0] // image_input_max_num,
                             rgba[1] * image_output_max_num_rgb[1] // image_input_max_num,
                             rgba[2] * image_output_max_num_rgb[2] // image_input_max_num]
            
            # get the binary representations of each, add to one string
            pixel_string += format(eight_bpp_rgb[0], "05b")
            pixel_string += format(eight_bpp_rgb[1], "06b")
            pixel_string += format(eight_bpp_rgb[2], "05b")

        # testing reverse byte order
        if (byte_swapped):
            pixel_string = "0b" + pixel_string[-8:] + pixel_string[:8]
        else:
            pixel_string = "0b" + pixel_string

        pixel_strings_to_add.append(pixel_string)

# create new image.h file
with open(sys.argv[2] + ".h", "w") as file:
    file.write("const unsigned short " + file_name + "[" + str(len(pixel_strings_to_add)) + "] = {\n")

    # write each pixel string, with a comma after, and line break after each 16
    for i, pixel_string in enumerate(pixel_strings_to_add):
        file.write(pixel_string + ", ")
        if (i % 8 == 7):
            file.write("\n")

    file.write("};")


