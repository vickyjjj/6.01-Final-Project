from Adafruit_AMG88xx import Adafruit_AMG88xx
import pygame
import os
import math
from time import sleep
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import griddata
from lib601.dist import *
from colour import Color
    
#low range of the sensor (this will be blue on the screen)
MINTEMP = 22

#high range of the sensor (this will be red on the screen)
MAXTEMP = 30

#how many color values we can have
COLORDEPTH = 1024

os.putenv('SDL_FBDEV', '/dev/fb1')
pygame.init()

#initialize the sensor
sensor = Adafruit_AMG88xx()

points = [(math.floor(ix / 8), (ix % 8)) for ix in range(0, 64)]
grid_x, grid_y = np.mgrid[0:7:32j, 0:7:32j]

#sensor is an 8x8 grid so lets do a square
height = 240
width = 240

#the list of colors we can choose from
blue = Color("indigo")
colors = list(blue.range_to(Color("red"), COLORDEPTH))

#create the array of colors
colors = [(int(c.red * 255), int(c.green * 255), int(c.blue * 255)) for c in colors]

displayPixelWidth = width / 30
displayPixelHeight = height / 30

lcd = pygame.display.set_mode((width, height))

lcd.fill((255,0,0))

pygame.display.update()
pygame.mouse.set_visible(False)

lcd.fill((0,0,0))
pygame.display.update()

#some utility functions
def constrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))

def map(x, in_min, in_max, out_min, out_max):
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

#let the sensor initialize
sleep(.1)

print("Show number 1")

counter = 0

array_var = []

while (counter < 30):
    #read the pixels
    print('prepare finger')
    sleep(1)
    pixels = sensor.readPixels()

    np_pixels = np.asarray(pixels)
    np_pixels = np_pixels.reshape((8,8))
    np_pixels = np_pixels.transpose()

    #subtract minimum of row from all elements of the row for greater range
    for a in range(8):
        row = np_pixels[a]
        minimum = row[np.argmin(row)]
        np_pixels[a] -= minimum
            
    array_var.append(np.var(np_pixels))
    
    pixels = [map(p, MINTEMP, MAXTEMP, 0, COLORDEPTH - 1) for p in pixels]
    
    #perdorm interpolation
    bicubic = griddata(points, pixels, (grid_x, grid_y), method='cubic')
    
    #draw everything
    for ix, row in enumerate(bicubic):
            for jx, pixel in enumerate(row):
                    pygame.draw.rect(lcd, colors[constrain(int(pixel), 0, COLORDEPTH- 1)], (displayPixelHeight * ix, displayPixelWidth * jx, displayPixelHeight, displayPixelWidth))
    
    pygame.display.update()

    counter += 1

filename = "variances_for_one.txt"
average = sum(array_var) / 30
print(average)
np.savetxt(filename, array_var)
