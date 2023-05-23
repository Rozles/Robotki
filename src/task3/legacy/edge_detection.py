from copy import copy
from email.mime import image
from unittest import result
from turtle import width
from matplotlib import pyplot as plt
import numpy as np
import cv2 as cv2
import math

def gauss(sigma):
    size = (2 * math.ceil(3 * sigma) + 1) // 2
    result = []
    for i in range(-size, size + 1):
        value = (1 / (np.sqrt(2 * math.pi) * sigma)) * np.exp(-(i * i) / (2 * sigma * sigma))
        result.append([value])
    result = np.array(result)
    return np.flip((result / np.sum(result)).T)

def gaussdx(sigma):
    size = (2 * math.ceil(3 * sigma) + 1) // 2
    result = []
    for i in range(-size, size + 1):
        value = (-1 / (np.sqrt(2 * math.pi) * sigma ** 3)) * i * np.exp(-(i ** 2) / (2 * sigma ** 2))
        result.append([value])
    result = np.array(result)
    return np.flip((result / np.sum(np.abs(result))).T)

def gradient_magnitude(image):
    sigma = 1.0
    G = gauss(sigma)
    D = gaussdx(sigma)
    I_x = cv2.filter2D(src=cv2.filter2D(src=image, ddepth=-1, kernel=G.T), ddepth=-1, kernel=D)
    I_y = cv2.filter2D(src=cv2.filter2D(src=image, ddepth=-1, kernel=G), ddepth=-1, kernel=D.T)
    return (np.sqrt(np.square(I_x) + np.square(I_y)), np.arctan2(I_y, I_x))

def findedges(image, sigma, theta):
    image_copy = image.copy()
    mag = gradient_magnitude(image)[0]
    image_copy[mag < theta] = 0
    image_copy[mag >= theta] = 1
    return image_copy

def non_maxima_suppresion(image):
    mag = gradient_magnitude(image)[0]
    mag = np.pad(mag, ((4,4),(4,4)), mode='constant', constant_values=0)
    dir = ((gradient_magnitude(image)[1]) * 180. / np.pi).astype(int)
    dir[dir < 0] += 180
    dir = np.pad(dir, ((4,4),(4,4)), mode='constant', constant_values=0)
    height,width = mag.shape
    result = np.zeros((height, width))
    for i in range(4, height - 4):
        for j in range(4, width - 4):
            current_dir = dir[i][j]
            current_mag = mag[i][j]
            vals = []

            #angle 0
            if (0 <= current_dir < 22.5) or (157.5 <= current_dir <= 180):
                vals.append(mag[i, j+1])
                vals.append(mag[i, j-1])
            #angle 45
            elif (22.5 <= current_dir < 67.5):
                vals.append(mag[i-1, j-1])
                vals.append(mag[i+1, j+1])
            #angle 90
            elif (67.5 <= current_dir < 112.5):
                vals.append(mag[i+1, j])
                vals.append(mag[i-1, j])
            #angle 135
            elif (112.5 <= current_dir < 157.5):
                vals.append(mag[i+1, j-1])
                vals.append(mag[i-1, j+1])

            vals = np.array(vals)
            
            if np.max(vals) > current_mag:
                result[i][j] = 0.0 
            else:
                result[i][j] = current_mag

    return result[3:height - 5, 3: width - 5]

def canny(nms, t_low, t_high):
    img = (np.where(nms < t_low, 0, nms) * 255).astype('uint8')
    nb_components, output, stats, centroids = cv2.connectedComponentsWithStats(img, cv2.CV_32S, connectivity=8)
    result = output.copy()
    # Loop over all components except for the backgrounds
    for i in np.unique(output):
        if i != 0:
            values = np.where(output == i, 1, 0)
            values = np.multiply(values, nms)
            # Check if any of the pixels is greater than t_high
            max = math.ceil(np.max(values) * 255)
            t = int(t_high * 255)
            if max >= t:
                result = np.where(result == i, 1, result)
            else:
                result = np.where(result == i, 0, result)       
    return result.astype(np.float64)

# # Load images
# museum_grey = Image.open('./images/museum.jpg').convert('L')  
# museum_grey = np.asarray(museum_grey)  
# museum_grey = (museum_grey / 255).astype(np.float64)

# def a():
#     theta_0 = findedges(museum_grey, 0, 0.04) 
#     theta_1 = findedges(museum_grey, 0, 0.06) 
#     theta_2 = findedges(museum_grey, 0, 0.08) 
#     theta_3 = findedges(museum_grey, 0, 0.10) 

#     plt.subplot(1,4,1)
#     plt.title("theta = 0.04", fontsize = 10)
#     plt.imshow(theta_0)
#     plt.set_cmap('gray') 
#     plt.subplot(1,4,2)
#     plt.title("theta = 0.06", fontsize = 10)
#     plt.imshow(theta_1)
#     plt.set_cmap('gray') 
#     plt.subplot(1,4,3)
#     plt.title("theta = 0.08", fontsize = 10)
#     plt.imshow(theta_2)
#     plt.set_cmap('gray') 
#     plt.subplot(1,4,4)
#     plt.title("theta = 0.10", fontsize = 10)
#     plt.imshow(theta_3)
#     plt.set_cmap('gray') 
#     plt.tight_layout()
#     plt.show()

# def b_c():
#     plt.subplot(2,2,1)
#     plt.title("Original", fontsize = 10)
#     plt.imshow(museum_grey)
#     plt.set_cmap('gray') 
#     plt.subplot(2,2,2)
#     plt.title("Thresholded (thr = 0.16)", fontsize = 10)
#     plt.imshow(findedges(museum_grey, 0, 0.16) )
#     plt.set_cmap('gray') 
#     plt.subplot(2,2,3)
#     plt.title("Nonmax. supp. (thr = 0.16)", fontsize = 10)
#     # Calculate non maxima suppression
#     nms = non_maxima_suppresion(museum_grey)
#     # Set a threshold
#     plt.imshow(np.where(nms >= 0.16, 1, 0))
#     plt.set_cmap('gray') 


#     plt.subplot(2,2,4)
#     plt.title("Hysteresis (high = 0.16, low = 0.04)", fontsize = 10)
#     c = canny(nms, 0.04, 0.16)
#     plt.imshow(c)
#     plt.set_cmap('gray') 

#     plt.tight_layout()
#     plt.show()

