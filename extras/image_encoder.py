# take in a png image, turn it into a image.h file
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

image_height = 170
image_width = 320

if (len(sys.argv) != 3):
    print("you need to pass in an image and a name")
    exit()

img = Image.open(sys.argv[1])
file_name = sys.argv[2]
array = np.asarray(img, dtype=np.uint64)

if (array.shape != (170, 320, 4)):
    print("image is not right shape, check if it is png or size")
    exit()

pixel_strings_to_add = []

# convert array from uint8 to 5, 6, 5 
for r in range(image_height):
    for c in range(image_width):
        rgba = array[r, c]
        # for each value, multiply by 32 (64 if green) then divide by 256
        eight_bpp_rgb = [rgba[0] * 32 // 256, rgba[1] * 64 // 256, rgba[2] * 32 // 256]
        
        # get the binary representations of each, add to one string
        pixel_string = "0b"
        pixel_string += format(eight_bpp_rgb[0], "05b")
        pixel_string += format(eight_bpp_rgb[1], "06b")
        pixel_string += format(eight_bpp_rgb[2], "05b")
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
