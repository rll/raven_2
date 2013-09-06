import numpy as np
import Image

def dim_to_int(dim):
    if dim == int(dim):
        return int(dim)
    else:
        raise Exception("Can't have a non-integer dimension")

DPI = 72

CALIB_X = 11
CALIB_Y = 7

X = CALIB_X + 1
Y = CALIB_Y + 1

SIZE = 0.5 # In inches
BORDER_SIZE = 1 # In inches, on each side of board.

SQUARE_PIXELS = dim_to_int(SIZE * DPI)
BORDER_PIXELS = dim_to_int(BORDER_SIZE * DPI)

width  = dim_to_int(X * SQUARE_PIXELS + BORDER_PIXELS*2)
height = dim_to_int(Y * SQUARE_PIXELS + BORDER_PIXELS*2)

page_width = width*1.0/DPI
page_height = height*1.0/DPI

print "Page size is: %.2f by %.2f" % (page_height, page_width)

data = np.zeros((height,width,3), dtype=np.uint8)

y_offset = 0

white = [255,255,255]
black = [0,0,0]

# Write top border:
for y in range(y_offset, y_offset + BORDER_PIXELS):
    for x in range(0, width):
        data[y,x] = white

y_offset += BORDER_PIXELS

# Write each row:
start_color = black
for row_number in range(Y):
    for y in range(y_offset, y_offset + SQUARE_PIXELS):
        # Write left border
        for x in range(0, BORDER_PIXELS):
                data[y,x] = white
        # Write right border:
        for x in range(width-BORDER_PIXELS, width):
                data[y,x] = white
        color = start_color
        for x_offset in range(BORDER_PIXELS, width-BORDER_PIXELS, SQUARE_PIXELS):
            for x in range(x_offset, x_offset+SQUARE_PIXELS):
                data[y,x] = color
            if color == black:
                color = white
            else:
                color = black
    y_offset += SQUARE_PIXELS

    if start_color == black:
        start_color = white
    else:
        start_color = black

# Write bottom border:
for y in range(y_offset, y_offset + BORDER_PIXELS):
    for x in range(0, width):
        data[y,x] = white

img = Image.fromarray(data)
img.save("checkerboard_%d_x_%d_%.2f_inches_%.2fin_x_%.2fin.png" % (CALIB_X, CALIB_Y, SIZE, page_width, page_height))
