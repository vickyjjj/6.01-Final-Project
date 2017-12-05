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

def basic_obs_model(state):
    if state == 0:
        triangle = triangle_dist(0, 1, hiLimit=1)
        uni = uniform_dist(range(8))
        return mixture(triangle, uni, .9)
    elif state == 1:
        triangle = triangle_dist(1, 2, loLimit = 1, hiLimit = 2)
        uni = uniform_dist(range(8))
        return mixture(triangle, uni, .6)
    elif state == 2:
        triangle = triangle_dist(2, 2, hiLimit=3)
        uni = uniform_dist(range(8))
        return mixture(triangle, uni, .6)
    elif state == 3:
        triangle = triangle_dist(3, 2, hiLimit=4)
        uni = uniform_dist(range(8))
        return mixture(triangle, uni, .6)
    else:
        return uniform_dist(range(8))
    
def update_prior(prior, peaks):
    for peak in peaks:
        print('peaks', peak)
        prior = bayes_rule(prior, basic_obs_model, sum(peak))
    return prior

def find_peaks(pixels):
    in_peak = False
    possibly_exiting = False
    
    np_pixels = np.asarray(pixels)
    np_pixels = np_pixels.reshape((8,8))
    np_pixels = np_pixels.transpose()

    maximum = np.argmax(pixels)
    minimum = np.argmin(pixels)

    peaks = np.zeros((8,8))
    for i in range(8):
        for j in range(8):
            if np_pixels[i][j] >= 26:
                num = abs(np_pixels[i][j-1] - np_pixels[i][j]) / np_pixels[i][j] * 100
                if num >= 5:
                    if not in_peak:
                        if peaks[i][j-1] != 1:
                            peaks[i][j] = 1
                            in_peak = True
                    else:
                        possibly_exiting = True
            else:
                if possibly_exiting:
                    possibly_exiting = False
                    in_peak = False
                elif in_peak:
                    in_peak = False
    
    temp_peaks = []

    #get rid of extraneous rows that do not provide valuable information
    for x in range(len(peaks)):
        #if this row is one of the "extreme" rows, check it
        if x < 3 or x > 4:
            row = peaks[x]
            counter = sum(row)
            if counter != 0 and counter < 5:
                temp_peaks.append(row)
        else:
            #otherwise, automatically append to array to be returned
            temp_peaks.append(row)
    return temp_peaks
    #return peaks

def find_confident(belief):
    print('max_prob:, ',belief.prob(belief.max_prob_elt()))
    if belief.prob(belief.max_prob_elt()) > .9:
        return belief.max_prob_elt()

def plot_data(data):
    np_pixels = np.asarray(data)
    np_pixels = np_pixels.reshape((8,8))
    np_pixels = np_pixels.transpose()
    for j in range(8):
        row = plt.plot(range(8), np_pixels[j][:], label = 'row ' + str(j))
    #plt.legend(handles=[plots])
    plt.title('Finger rows')
    plt.show()   
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

prior = DDist({0:.25, 1:.25, 2:.25, 3:.25})
while(1):
    #read the pixels
    print('Show a number: ')
    pixels = sensor.readPixels()
    
    #plot_data(pixels)
    time.sleep(1)
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
