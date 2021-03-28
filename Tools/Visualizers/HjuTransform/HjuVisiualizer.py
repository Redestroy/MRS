import matplotlib.pyplot as plt
import HjuTransform
from PIL import Image
from numpy import asarray
import numpy as np

class pixel:
    def __init__(self, x1, y1, value):
        self.x = x1
        self.y = y1
        self.value = value


def main():
    MAX_DIST = 512
    fig = plt.figure()
    ax = fig.add_axes([0, 0, 2*3.14159, MAX_DIST])
    #get picture
    #change picture to matrix
    pixels = list()
    # load the image and convert into
    # numpy array
    img = Image.open('Sample.png')

    # asarray() class is used to convert
    # PIL images into NumPy arrays
    img_data = asarray(img)

    # <class 'numpy.ndarray'>
    print(type(img_data))

    #  shape
    size = img_data.shape
    x = 0
    y = 0
    for val in np.nditer(img_data):
        pixels.append(pixel(x, size[1] - y, val))
        if x + 1 == size[0]:
            x = 0
            y += 1
        else:
            x += 1
    print(pixels)
    #run hju transform
    array1 = HjuTransform.HjuTransform(0.15, 10, size[0]+size[1]/2, 2*3.14159, pixels)
    #array2 = HjuTransform.HjuTransform(0.15, 10, size[0]+size[1]/2, 2*3.14159, pixels2)
    #plot hju tranform result
    ang = np.linspace(0, 2*3.14159, 2*3.14159/0.15)
    dist = array1[0]
    plt.plot(ang, dist)
    plt.show()

if __name__ == "__main__":
    main()
