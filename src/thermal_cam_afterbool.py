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

##import sys
##sys.path.append("/usr/lib/python3/dist-packages")
##import espeak

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
        prior = bayes_rule(prior, basic_obs_model, sum(peak))
    return prior

def find_peaks(pixels):
    
    np_pixels = np.asarray(pixels)
    np_pixels = np_pixels.reshape((8,8))
    np_pixels = np_pixels.transpose()

    average = 0

    #subtract minimum of row from all elements of the row for greater range
    for a in range(8):
        row = np_pixels[a]
        minimum = row[np.argmin(row)]
        np_pixels[a] -= minimum
        for b in np_pixels[a]:
            average += b

    #find total average of picture
    average /= 64

    variance = np.var(np_pixels)
    print(variance)

    peaks = np.zeros((8,8))

    if variance < 5:
        return peaks
    
    for i in range(8):
        for j in range(8):
            if np_pixels[i][j] >= average:
                peaks[i][j] = 1


    #print("original reading", peaks)

    in_region = False
    temp_peaks = []

    counter = 0

    for k in range(8):
        for l in range(8):
            el = peaks[k][l]

            #if the element is a 1
            if el == 1:
                counter += 1

                #if we are in a region
                if in_region:
                    #change current element value so that the region stays at length 1
                    peaks[k][l] = 0
                    #print("Zero changed at ", k, l)
                else:
                    #otherwise, we have entered a new region; set boolean to true
                    in_region = True
            #else if the element is a 0
            else:
                in_region = False
        #check if row's values are significant to be counted
        
        if k < 3 or k > 4:
            row = peaks[k]
            #print("Counter, row ", k, ":", counter)
            if counter != 0 and counter < 4:
                temp_peaks.append(peaks[k])
                #print("appended: ", k, row)
        else:
            temp_peaks.append(peaks[k])
            #print("appended: ", k, row)
        #reset variables
        counter = 0
        in_region = False
        
    #for i in temp_peaks:
        #print(i)
    
    return temp_peaks
    #return peaks

def find_confident(belief):
    print('max_prob:, ',belief.prob(belief.max_prob_elt()))
    if belief.prob(belief.max_prob_elt()) > .99:
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

#print("Show number")

three_fingers = np.zeros(64)

prior = DDist({0:.25, 1:.25, 2:.25, 3:.25})
#os.system("espeak 'Please, show a number.'")
while(1):
    #read the pixels
    
    pixels = sensor.readPixels()
    
    #plot_data(pixels)
    time.sleep(1)
    peaks = find_peaks(pixels)
    prior = update_prior(prior, peaks)
    elt = find_confident(prior)
    file = open("refills.txt", "w")
    if elt == None:
        file.write("0")
    else:
        file.write(str(elt))
    file.close()
    string_elt = str(elt)
##    if elt != None:
##        print('Your number is %d'%elt)
##
##        #show display again after printing number of fingers for debugging/analysis
##        pixels = [map(p, MINTEMP, MAXTEMP, 0, COLORDEPTH - 1) for p in pixels]
##        bicubic = griddata(points, pixels, (grid_x, grid_y), method='cubic')
##        for ix, row in enumerate(bicubic):
##            for jx, pixel in enumerate(row):
##                    pygame.draw.rect(lcd, colors[constrain(int(pixel), 0, COLORDEPTH- 1)], (displayPixelHeight * ix, displayPixelWidth * jx, displayPixelHeight, displayPixelWidth))
##        pygame.display.update()
##
##        time.sleep(3)
##        
##        break
    
    pixels = [map(p, MINTEMP, MAXTEMP, 0, COLORDEPTH - 1) for p in pixels]
    
    #perdorm interpolation
    bicubic = griddata(points, pixels, (grid_x, grid_y), method='cubic')
    
    #draw everything
    for ix, row in enumerate(bicubic):
            for jx, pixel in enumerate(row):
                    pygame.draw.rect(lcd, colors[constrain(int(pixel), 0, COLORDEPTH- 1)], (displayPixelHeight * ix, displayPixelWidth * jx, displayPixelHeight, displayPixelWidth))
    
    pygame.display.update()
