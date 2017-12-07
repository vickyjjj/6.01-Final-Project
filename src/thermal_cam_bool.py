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
import sys
sys.path.append("/usr/lib/python3/dist-packages")
import espeak

#average value whole image threshold to compare abs temp

def basic_obs_model(state):
    if state == 0:
        triangle = triangle_dist(0, 1, hiLimit=1)
        uni = uniform_dist(range(8))
        return mixture(triangle, uni, .4)
    elif state == 1:
        triangle = triangle_dist(1, 2, loLimit = 1, hiLimit = 2)
        uni = uniform_dist(range(8))
        return mixture(triangle, uni, .4)
    elif state == 2:
        triangle = triangle_dist(2, 2, hiLimit=3)
        uni = uniform_dist(range(8))
        return mixture(triangle, uni, .4)
    elif state == 3:
        triangle = triangle_dist(3, 2, hiLimit=4)
        uni = uniform_dist(range(8))
        return mixture(triangle, uni, .4)
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

    tot_max = np.argmax(np_pixels)
    tot_min = np.argmin(np_pixels)
    print(tot_max, tot_min)

    #for using percentage difference, do not use absolute temperature
    #subtract minimum of row from all elements of the row for greater range
    for a in range(8):
        row = np_pixels[a]
        minimum = row[np.argmin(row)]
        np_pixels[a] -= minimum

    peaks = np.zeros((8,8))

    region_counter = 0
    in_peak = False
    possibly_exiting = False
    
    for i in range(8):
        minimum = np.argmin(np_pixels[i])
        for j in range(8):
            #calculate absolute difference from previous cell
            num = abs(np_pixels[i][j-1] - np_pixels[i][j])

            #if the absolute difference is significant
            if num >= 3.5:

                #if we aren't in a region
                if not in_peak:

                    #if the previous cell isn't 1 and the region counter is 0
                    if peaks[i][j-1] != 1:
                        #this is a significant finger
                        peaks[i][j] = 1

                        #we are now in a region
                        in_peak = True
                        region_counter += 1
                #otherwise if we are in a region
                else:
                    in_peak = False
                    if region_counter >= 5:
                        print("large region", region_counter)
                        peaks[i] = np.zeros((8))
                    region_counter = 0
            #else if the percentage difference is not significant
            else:
                if in_peak and np_pixels[i][j] >= 3:
                    region_counter += 1
                else:
                    in_peak = False
                    if region_counter >= 5:
                        print("large region", region_counter)
                        peaks[i] = np.zeros((8))
                    region_counter = 0
                    
        #reset variables for next row
        region_counter = 0
        in_peak = False
    
    temp_peaks = []
    print(peaks)
    time.sleep(1)

    #get rid of extraneous rows that do not provide valuable information
    for x in range(len(peaks)):
        #if this row is one of the "extreme" rows, check it
        if x < 3 or x > 4:
            row = peaks[x]
            counter = sum(row)
            if counter != 0 and counter < 5:
                temp_peaks.append(row)
                print("appended row ", x, row)
        else:
            #otherwise, automatically append to array to be returned
            temp_peaks.append(row)
            print("appended row ", x)
    return temp_peaks
    #return peaks

def find_confident(belief):
    print('max_prob:, ',belief.prob(belief.max_prob_elt()))
    if belief.prob(belief.max_prob_elt()) > .99:
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

#os.system("espeak 'Please, show a number.'")

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
    elt = find_confident(prior)
    string_elt = str(elt)
    message = "Your number is " + string_elt
    if elt != None:
        print('Your number is %d'%elt)
        #os.system('espeak "{}"'.format(message))

        #show display again after printing number of fingers
        pixels = [map(p, MINTEMP, MAXTEMP, 0, COLORDEPTH - 1) for p in pixels]
        bicubic = griddata(points, pixels, (grid_x, grid_y), method='cubic')
        for ix, row in enumerate(bicubic):
            for jx, pixel in enumerate(row):
                    pygame.draw.rect(lcd, colors[constrain(int(pixel), 0, COLORDEPTH- 1)], (displayPixelHeight * ix, displayPixelWidth * jx, displayPixelHeight, displayPixelWidth))
        pygame.display.update()
        
        time.sleep(10000)
        
        break
    pixels = [map(p, MINTEMP, MAXTEMP, 0, COLORDEPTH - 1) for p in pixels]
    
    #perdorm interpolation
    bicubic = griddata(points, pixels, (grid_x, grid_y), method='cubic')
    
    #draw everything
    for ix, row in enumerate(bicubic):
            for jx, pixel in enumerate(row):
                    pygame.draw.rect(lcd, colors[constrain(int(pixel), 0, COLORDEPTH- 1)], (displayPixelHeight * ix, displayPixelWidth * jx, displayPixelHeight, displayPixelWidth))
    
    pygame.display.update()
