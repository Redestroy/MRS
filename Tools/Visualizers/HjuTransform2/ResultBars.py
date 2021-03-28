import matplotlib.pyplot as plt
import HjuTransform
from PIL import Image
from numpy import asarray
import numpy as np
import math

class pixel:
    def __init__(self, x1, y1, value):
        self.x = x1
        self.y = y1
        self.value = value


def main():
    MAX_DIST = 512
    pixels = list()
    # Dabū pikseļu datus
    img1 = Image.open('Karte1.png')
    img2 = Image.open('Karte2.png')
    img_data1 = asarray(img1)
    img_data2 = asarray(img2)

    size = img_data1.shape
    #print(size)
    #print(img_data)
    x = 0
    y = 0
    # Parāda attēlu
    plt.imshow(img_data1)
    plt.show()
    for val in np.nditer(img_data1):
        pixels.append(pixel(x,size[1] - y,val))
        if x + 1 == size[0]:
            x = 0
            y += 1
        else:
            x += 1

    #run hju transform
    Map1_HT = HjuTransform.HjuTransform3(0.10, 1, 100, int(3.14159/0.10), img_data1)
    Map1_HS = HjuTransform.HjuSpectrum(0.10, 1, 100, int(3.14159/0.10), Map1_HT)
    Map2_HT = HjuTransform.HjuTransform3(0.10, 1, 100, int(3.14159/0.10), img_data2)
    Map2_HS = HjuTransform.HjuSpectrum(0.10, 1, 100, int(3.14159/0.10), Map2_HT)
    #array2 = HjuTransform.HjuTransform(0.15, 10, size[0]+size[1]/2, 2*3.14159, pixels2)
    #plot hju tranform result
    print(Map1_HS)
    #plt.imshow(Map1_HT)
    #plt.show()
    ang = np.linspace(0, 360/2, int(3.14159/0.10))
    dist = Map1_HS
    plt.xlabel("leņķis")
    plt.ylabel("Spektrs")
    plt.title('Hjū spektra atkarība no leņķa kartei 1')
    plt.plot(ang, dist)
    plt.show()

    dist = Map2_HS
    plt.xlabel("leņķis")
    plt.ylabel("Spektrs")
    plt.title('Hjū spektra atkarība no leņķa kartei 2')
    plt.plot(ang, dist)
    plt.show()
    #Circular cor. Map1 Map2
    CircCor = HjuTransform.HjuCircularCorrelation(0.10, int(3.14159/0.10), Map1_HS, Map2_HS)
    dist = CircCor
    plt.xlabel("rotācija")
    plt.ylabel("Spektru korelācija")
    plt.title('Hjū spektru korelācija starp kartēm 1 un 2')
    plt.plot(ang, dist)
    plt.show()

    # Rotate second map
    rotation = np.argmax(CircCor)*0.1
    img_data2_rot = HjuTransform.Rotate(img_data2, rotation)
    plt.title('Karte pēc pēc rotācijas')
    plt.imshow(img_data2_rot)
    plt.show()

    # X spektrs
    X_spectrum_Map1 = HjuTransform.X_Spectrum(img_data1)
    X_spectrum_Map2 = HjuTransform.X_Spectrum(img_data2_rot)
    # Y spektrs
    Y_spectrum_Map1 = HjuTransform.Y_Spectrum(img_data1)
    Y_spectrum_Map2 = HjuTransform.Y_Spectrum(img_data2_rot)
    # Translācijas korelācija
    dist1 = np.linspace(0, 50)
    XTranslateCor1 = HjuTransform.HjuLinearCorrelation(1, 50, X_spectrum_Map1, X_spectrum_Map2)
    dist = XTranslateCor1
    plt.xlabel("translācija")
    plt.ylabel("Spektru korelācija X virzienā 1")
    plt.title('Translācijas spektru korelācija starp kartēm 1 un 2')
    plt.plot(dist1, dist)
    plt.show()
    XTranslateCor2 = HjuTransform.HjuLinearCorrelation(1, 50, X_spectrum_Map2, X_spectrum_Map1)
    dist = XTranslateCor2
    plt.xlabel("translācija")
    plt.ylabel("Spektru korelācija X virzienā 2")
    plt.title('Translācijas spektru korelācija starp kartēm 1 un 2')
    plt.plot(dist1, dist)
    plt.show()

    dist2 = np.linspace(0, 50)
    YTranslateCor1 = HjuTransform.HjuLinearCorrelation(1, 50, Y_spectrum_Map1, Y_spectrum_Map2)
    dist = YTranslateCor1
    plt.xlabel("translācija")
    plt.ylabel("Spektru korelācija Y virzienā 3")
    plt.title('Translācijas spektru korelācija starp kartēm 1 un 2')
    plt.plot(dist2, dist)
    plt.show()
    YTranslateCor2 = HjuTransform.HjuLinearCorrelation(1, 50, Y_spectrum_Map2, Y_spectrum_Map1)
    dist = YTranslateCor2
    plt.xlabel("translācija")
    plt.ylabel("Spektru korelācija Y virzienā 4")
    plt.title('Translācijas spektru korelācija starp kartēm 1 un 2')
    plt.plot(dist2, dist)
    plt.show()

    # calculate new size
    size_img1 = np.shape(img_data1)
    size_img2 = np.shape(img_data1)


    #print(translation)
    rotation = np.argmax(CircCor)*0.1
    rot = np.array([[math.cos(rotation), -math.sin(rotation)], [math.cos(rotation), math.sin(rotation)]])
    new_size = ((size_img1[0]+size_img2[0])*2, (size_img1[0]+size_img2[0])*2)

    # create array for new image
    peaks = 3
    result_img = list()
    translation = np.zeros([2, 1])
    best_x_transl = HjuTransform.GetLocalMaximaArgs(XTranslateCor1, peaks, 3)
    print("X")
    print(best_x_transl)
    best_y_transl = HjuTransform.GetLocalMaximaArgs(YTranslateCor2, peaks, 3)
    print("Y")
    print(best_y_transl)
    # Insert map 1 in corner
    for n in range(0, peaks):
        for m in range(0, peaks):
            translation[0, 0] = best_x_transl[n][0]
            translation[1, 0] = best_y_transl[m][0]
            result_img.append(np.ones([new_size[0], new_size[1]]))
            for x in range(0, size_img1[0]):
                for y in range(0, size_img1[1]):
                    if img_data1[x, y] < 1:
                        result_img[3*n+m][x, y] = img_data1[x, y]
    # insert map 2 with translation

            for x in range(0, size_img2[0]):
                for y in range(0, size_img2[1]):
                    if img_data2_rot[x, y] < 1:
                        result_img[3*n+m][int(translation[0, 0])+x, int(translation[1, 0])+y] = 0
    plt.imshow(result_img[0])
    plt.show()
    plt.imshow(result_img[1])
    plt.show()
    plt.imshow(result_img[2])
    plt.show()
    plt.imshow(result_img[3])
    plt.show()
    plt.imshow(result_img[4])
    plt.show()
    plt.imshow(result_img[5])
    plt.show()
    plt.imshow(result_img[6])
    plt.show()
    plt.imshow(result_img[7])
    plt.show()
    plt.imshow(result_img[8])
    plt.show()




if __name__ == "__main__":
    main()
