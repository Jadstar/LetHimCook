import cv2 as cv
import numpy as np
from ipdb import set_trace as pst
import matplotlib.pyplot as plt

con = 0
# bgr colour values
black = [ 0,0,0 ]
brown = [ 0,51,102 ]
red = [ 0,0,255 ]
orange = [ 0,128,255 ]
yellow = [ 0,255,255 ]
green = [ 0,255,0 ]
blue = [ 255,0,0 ]
purple = [ 255,0,127 ]
grey = [ 128,128,128 ]
white = [ 255,255,255 ]
gold = [ 55, 175, 212 ]

colours = [
        [(0, 0, 0)      , (23, 165, 65)     ,'BLACK' , 0, black ],
        [(6, 8, 0)      , (12, 255, 160)    ,'BROWN' , 1, brown ],
        [(0, 164, 130)  , (9, 255,255)      ,'RED'   , 2, red   ],
        [(11, 172, 103) , (24, 255, 255)    ,'ORANGE', 3, orange],
        [(30, 170, 100) , (40, 250, 255)  ,'YELLOW', 4, yellow  ],
        [(33, 76, 0)    , (57, 225, 255)    ,'GREEN' , 5, green ],
        [(80, 0, 0)     , (117, 255, 255)   ,'BLUE'  , 6, blue  ],
        [(120, 40, 100) , (140, 250, 220) ,'PURPLE', 7, purple  ],
        [(0, 0, 50)     , (179, 50, 80)   ,'GREY'  , 8, grey    ],
        [(0, 0, 90)     , (179, 15, 250)  ,'WHITE' , 9, white   ],
        [(17, 107, 0)   , (35, 158, 182)    ,'GOLD'  , 0, gold  ],
]

red_top_lower = (0, 164, 0)
red_top_upper = (9, 255, 255)
# min_area = 40

def read_img(path: str, scale=False,  grey: bool=False, blur: int=0, k: int = 7) -> np.ndarray:
    """Read image and display in new window"""
    
    img = cv.imread(path)

    if scale:
        img = rescale_frame(img, scale)

    if grey:
        img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    if blur == 1:
        # guassian blur
        img = cv.GaussianBlur(img, (k,k), cv.BORDER_DEFAULT)
        # kernel numbers always odd
    elif blur == 2:
        # averge blur of center px based on kernel
        img = cv.blur(img, (k,k))
    elif blur == 3:
        # median blur
        img = cv.medianBlur(img, k)
    elif blur == 4:
        # bilateral blur - best application
        img = cv.bilateralFilter(img, 5, 15, 15)
        # img, px neighbourhood diameter (not k), sigma colour (larger more colours considered), sigma space (px further out influential in blurring)

    # has name of window as path name, can change later
    cv.imshow(path, img)
    # cv.waitKey(0)

    return img


def rescale_frame(frame, scale=1):
    """Rescale image frame"""

    print(f'orig width x height: {frame.shape[1]} x {frame.shape[0]}')

    width = int(frame.shape[1] * scale)
    height = int(frame.shape[0] * scale)
    dimensions = (width, height)

    print(f'new width x height: {width} x {height}')

    return cv.resize(frame, dimensions, interpolation=cv.INTER_AREA)


def find_edges(img):
    """
    Find edges in the image
        - This is considered best practice
    """    
    # numbers are upper and lower threshold values for pixel gradients - prevent hardcoding later
    canny = cv.Canny(img, 125, 175)
    # cv.imshow('edges', canny)
    # cv.waitKey(0)

    return canny


def find_contours(img):
    """
    Find contours and their heirachies from edges in image and draw them on a blank image
        - Can be issues with this method combined with threshold
        - Simplifies things too much
    """
    # change to RETR_EXTERNAL for all external contours TREE for heirarchy
    # change to CHAIN_APPROX_SIMPLE to compress line points into 2 end points
    contours, heirarchies = cv.findContours(img, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    print(f'contours found: {len(contours)}')

    # need to have 3 colour channels to draw (greyscale only has 1)
    # blank = cv.cvtColor(np.zeros(img.shape, dtype='uint8'), cv.COLOR_GRAY2RGB)
    # cv.drawContours(blank, contours, -1, (255,255,255), 1)
    # cv.imshow('drawn contours', blank)

    return contours


def binary_img(img):
    """Binarise images based on pixel threshold values"""
    ret, thresh = cv.threshold(img, 125, 255, cv.THRESH_BINARY)
    # cv.imshow('binary', thresh)

    return thresh


def colour_spaces(img, hsv=False, lab=False, rgb=False):
    """Transform images to different colour spaces"""
    if hsv:
        img = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        # cv.imshow('hsv', img)
    if lab:
        img = cv.cvtColor(img, cv.COLOR_BGR2LAB)
        # cv.imshow('lab', img)
    if rgb:
        img = cv.cvtColor(img, cv.COLOR_BGR2RGB)
        # cv.imshow('rgb', img)

    return img


def colour_split(img: np.ndarray, colour_grey: bool = False, channels: list[bool] = [0,0,0]):
    """Split an image into its colour channels, which shows pixel intensity
        - Variables to show colour densities in greyscale and/or colour
    """
    try:
        b, g, r = cv.split(img)
    except ValueError:
        raise ValueError("Image not accepted, check image is coloured")
    
    if colour_grey:
        '''
        show colour densities in greyscale
        '''        # cv.imshow('blue', b)
        # cv.imshow('green', g)
        # cv.imshow('red', r)
        
    # to merge back into coloured image
    # merge = cv.merge([b,g,r])
    # cv.imshow('merged channels', merge)
 
    if channels:
        # show colour densities in colour
        blank = cv.cvtColor(np.zeros(img.shape, dtype='uint8'), cv.COLOR_BGR2GRAY)
    if channels[0] == 1:
        blue = cv.merge([b, blank, blank])
        # cv.imshow('blue channel', blue)

    if channels[1] == 1:    
        green = cv.merge([blank, g, blank])
        # cv.imshow('green channel', green)

    if channels[2] == 1:
        red = cv.merge([blank, blank, r])
        # cv.imshow('red channel', red)


def bitwise(img_1, img_2, opt: str):
    """Bitwise operations with two images"""

    # intersecting regions
    if opt == 'and':
        bitwise = cv.bitwise_and(img_1, img_2)

    # intersecting and non intersecting regions 
    if opt == 'or':
        bitwise = cv.bitwise_or(img_1, img_2)

    # non intersecting regions
    if opt == 'xor':
        bitwise = cv.bitwise_or(img_1, img_2)

    # inversion of colour 
    if opt == 'not':
        bitwise = cv.bitwise_or(img_1)
        
    # cv.imshow(f'bitwise {opt}', bitwise)


def test_masking(img, opt: str = 'rec' or 'cir', crop=False):
    """Masking an image with a shape"""

    if type(img) == str:
        img = read_img(img)
    elif type(img) == np.ndarray:
        img = img
    
    blank = np.zeros(img.shape[:2], dtype='uint8')

    if opt == 'rec':
        mask = cv.rectangle(blank, (0 + 30, img.shape[0]//2 + 20), (img.shape[1], img.shape[0]//2 + 10), 255, -1)
        # mask = cv.rectangle(blank, (img.shape[2]//5, img.shape[0]//4), (img.shape[1]//2 + 170, img.shape[0]//3 + 10), 255, -1)

    if opt == 'cir':
    # can change shapes and positions
        mask = cv.circle(blank, (img.shape[1]//2, img.shape[0]//2), 200, 255, -1)
    # cv.imshow('mask', mask)

    masked = cv.bitwise_and(img, img, mask=mask)
    # cv.imshow('masked image', masked)

    return masked


def histogram(img, mode: str, mask: bool = False):
    """Histogram for greyscale and rgb images to show pixel density, can include masking"""
    plt.figure
    plt.title(f'{mode} histogram')
    # bins are intervals of px intensity
    plt.xlabel('bins')
    plt.ylabel('# of pixels')

    if type(img) == str and mode == 'grey':
        img = read_img(img, grey=True)
    elif type(img) == str:
        img = read_img(img)
    elif type(img) == np.ndarray:
        img = img
    
    if mask:
        mask = test_masking(img)
    else:
        mask = None

    if mode == 'grey':
        hist = cv.calcHist([img], [0], mask, [256], [0,256])
        plt.plot(hist)
        plt.xlim([0,256])
        plt.show()
    
    if mode == 'colour':
        colours = ('b', 'g', 'r')
        for i,col in enumerate(colours):
            hist = cv.calcHist([img], [i], mask, [256], [0,256])
            plt.plot(hist, color=col)
            plt.xlim([0,256])
    
    if mode == 'hsv':
        h_bins = 16
        s_bins = 16
        hist_size = [h_bins, s_bins]
        h_range = [0, 180]
        s_range = [0, 256]
        ranges = h_range + s_range
        channels = [0, 1]
        
        hist = cv.calcHist([img], channels, mask, hist_size, ranges, accumulate=False)
        hist_norm = cv.normalize(hist, None, alpha=0, beta=1, norm_type=cv.NORM_MINMAX)
        
        hist_img = np.zeros((h_bins * s_bins, 1), dtype=np.float32)
        for i in range(h_bins):
            for j in range(s_bins):
                bin_val = hist_norm[i, j]
                hist_img[i * s_bins + j] = bin_val
        
        plt.plot(hist_img)
        plt.xlim([0, h_bins * s_bins])
    
    plt.show()


def adapt_thresh(input, k: int = 11, c: int = 5):
    """Create binary image by comparing pixels to a specific threshold determined by computer via thresholding method"""
    
    if type(input) == str:
        img = read_img(input, grey=True)
    elif type(input) == np.ndarray:
        img = input

    # adaptive thresholding
    adaptive_thresh = cv.adaptiveThreshold(img, 255, cv.ADAPTIVE_THRESH_MEAN_C, cv.THRESH_BINARY, k, c)
    # can use other 
    # cv.imshow('adaptive thresholded', adaptive_thresh)
    return adaptive_thresh


def gradients(path, lap=False, sobel=False):
    """Alternative edge detection method, both less advanced than Canny"""
    img = read_img(path, grey=True)
    
    if lap:
        # Laplacian
        # gradient of greyscale px converted to an image
        lap = cv.Laplacian(img, cv.CV_64F)
        lap = np.uint8(np.absolute(lap))
        # cv.imshow('lapacian', lap)

    if sobel:
        # Sobel
        # Computes x and y direction gradients
        sobelx = cv.Sobel(img, cv.CV_64F, 1, 0)
        sobely = cv.Sobel(img, cv.CV_64F, 0, 1)
        combined_sobel = cv.bitwise_or(sobelx, sobely)
        # cv.imshow('combined sobel', combined_sobel)
        # cv.imshow('y sobel', sobely)
        # cv.imshow('x sobel', sobelx)

"""
path = 'img/100-ohm-res.PNG'
img = read_img(path, grey=True)
edges = find_edges(img)

# thresh and find_contours only work with greyscale
thresh = binary_img(img)
find_contours(thresh)

# colour functions - only work with colour
img = read_img(path)
colour_spaces(img, hsv=True, lab=True, rgb=True)
colour_split(img)
colour_split(img, colour_grey=True, channels=[1,1,1])

grey = read_img(path, grey=True)
mask = test_masking(grey, opt='rec')
x,y,w,h = cv.boundingRect(mask)
img = img[y:y+h, x:x+w]
adapt_thresh(path)
gradients(path, lap=True, sobel=True)
histogram(img, mode='colour')

# delay to keep dialogue windows open
cv.waitKey(0)
cv.destroyAllWindows()
"""