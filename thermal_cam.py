from Adafruit_AMG88xx import Adafruit_AMG88xx
import pygame
import os
import math
import time
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import griddata
from lib601.dist import *
from colour import Color

##def dumm_obs_model(peak_dict):
##    len_peak = len(peak_dict)
##    if len_peak == 0:
##        return DDist({0:.8, 1:.2/3, 2:.2/3, 3:.2/3})
##    elif len_peak == 1:
##        return DDist({0:.03, 1:.8, 2:.1, 3:.07})
##    elif len_peak == 2:
##        return DDist({0:.01, 1:.2, 2:.6, 3:.19})
##    elif len_peak == 3:
##        return DDist({0:0, 1:.1, 2:.3, 3:.6})
##    else:
##        return DDist({0:0, 1:0, 2:.3, 3:.7})

def basic_obs_model(state):
    if state == 0:
        triangle = triangle_dist(0, 1, loLimit=0)
        uni = uniform_dist(range(8))
        return mixture(triangle, uni, .9)
    elif state == 1:
        triangle = triangle_dist(1, 2)
        uni = uniform_dist(range(8))
        return mixture(triangle, uni, .9)
    elif state == 2:
        triangle = triangle_dist(2, 2)
        uni = uniform_dist(range(8))
        return mixture(triangle, uni, .9)
    elif state == 3:
        triangle = triangle_dist(3, 2, hiLimit=4)
        uni = uniform_dist(range(8))
        return mixture(triangle, uni, .9)
    else:
        return uniform_dist(range(8))
    
def update_prior(prior, peaks):
    updated_prior = bayes_rule(prior, basic_obs_model, sum(peaks))
    return updated_prior

def find_peaks(pixels):
    np_pixels = np.asarray(pixels)
    np_pixels = np_pixels.reshape((8,8))
    np_pixels_avg = np.average(np_pixels, axis=1)
    peaks = np.zeros(8)
    for i in range(len(np_pixels_avg)):
        if np_pixels_avg[i]>=28.2:
            peaks[i] = 1
    return peaks

def find_confident(belief):
    print('max_prob:, ',belief.prob(belief.max_prob_elt()))
    if belief.prob(belief.max_prob_elt()) > .9:
        return belief.max_prob_elt()
    
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
time.sleep(.1)

print("Show number 1")

three_fingers = np.zeros(64)

##for i in range(10):
##    time.sleep(2)
##    pixels = np.asarray(sensor.readPixels())
##    
####    for x in range(len(three_fingers)):
####        three_fingers[x] += pixels[x]
##
##    
##
##    #set up the picture
##    pixels = [map(p, MINTEMP, MAXTEMP, 0, COLORDEPTH - 1) for p in pixels]
##	
##    #perdorm interpolation
##    bicubic = griddata(points, pixels, (grid_x, grid_y), method='cubic')
##    
##    #draw everything
##    for ix, row in enumerate(bicubic):
##            for jx, pixel in enumerate(row):
##                    pygame.draw.rect(lcd, colors[constrain(int(pixel), 0, COLORDEPTH- 1)], (displayPixelHeight * ix, displayPixelWidth * jx, displayPixelHeight, displayPixelWidth))
##    
##    pygame.display.update()
    
##three_fingers /= 10
##
##one_finger = [23.775, 23.95, 24.475, 23.75, 23.2, 23.875, 23.6, 24.175, 23.6, 23.95, 24.925, 25.8, 26.55, 26.725, 25.95, 25.9, 24.25, 27.625, 29.625, 30.475, 31.15, 31.5, 31.25, 31.075, 25.4, 29.725, 29.775, 29.75, 29.675, 30.85, 30.7, 31.0, 24.525, 25.825, 26.025, 25.825, 25.275, 26.65, 28.225, 27.675, 22.9, 23.0, 23.475, 23.65, 23.525, 24.2, 26.675, 26.775, 23.125, 25.15, 23.45, 22.925, 22.95, 23.725, 24.55, 25.975, 23.4, 24.45, 26.175, 23.775, 23.225, 23.325, 23.525, 24.05]
##one_finger = np.asarray(one_finger)
##one_finger = one_finger.reshape((8,8))
##  
##two_fingers = [23.825, 24.3, 25.4, 25.575, 25.8, 26.725, 26.55, 26.725, 23.35, 24.725, 28.75, 31.125, 31.325, 31.525, 31.275, 31.575, 23.25, 25.25, 29.225, 28.675, 27.875, 28.0, 28.8, 29.4, 22.95, 23.475, 23.275, 23.45, 24.025, 27.025, 28.975, 31.375, 23.15, 23.375, 24.825, 26.625, 29.175, 31.4, 32.15, 31.5, 23.2, 25.525, 30.375, 31.525, 31.275, 30.375, 29.4, 27.125, 23.85, 27.425, 28.825, 27.65, 25.9, 25.175, 25.825, 27.075, 23.95, 25.675, 27.775, 25.05, 24.5, 24.15, 24.3, 25.3]
##two_fingers = np.asarray(two_fingers)
##two_fingers = two_fingers.reshape((8,8))
##
##three_fingers = three_fingers.reshape((8,8))
##three_fingers_avg = np.average(three_fingers, axis=1)
##one_finger_avg = np.average(one_finger, axis=1)
##two_fingers_avg = np.average(two_fingers, axis=1)
##plt.plot(range(8), one_finger_avg, '-')
##plt.plot(range(8), two_fingers_avg, '--')
##plt.plot(range(8), three_fingers_avg, '*')
##plt.show()
##for i in range(8):
##    n = i*8
##    plt.plot(range(8), one_finger[i:i+8])
##plt.show()
##for i in range(8):
##    n = i*8
##    plt.plot(range(8) ,two_fingers[i:i+8])
##plt.show()
    
##
prior = DDist({0:.25, 1:.25, 2:.25, 3:.25})
while(1):
    #read the pixels
    print('Show a number: ')
    pixels = sensor.readPixels()
    peaks = find_peaks(pixels)
    prior = update_prior(prior, peaks)
    print(prior)
    elt = find_confident(prior)
    if elt != None:
        print('Your number is %d'%elt)
        break
    pixels = [map(p, MINTEMP, MAXTEMP, 0, COLORDEPTH - 1) for p in pixels]
    
    #perdorm interpolation
    bicubic = griddata(points, pixels, (grid_x, grid_y), method='cubic')
    
    #draw everything
    for ix, row in enumerate(bicubic):
            for jx, pixel in enumerate(row):
                    pygame.draw.rect(lcd, colors[constrain(int(pixel), 0, COLORDEPTH- 1)], (displayPixelHeight * ix, displayPixelWidth * jx, displayPixelHeight, displayPixelWidth))
    
    pygame.display.update()
