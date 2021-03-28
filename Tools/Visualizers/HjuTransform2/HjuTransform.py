import math
import numpy as np

def HjuTransform(step_angle, step_distance, steps_distance, steps_angle, pixel_array):
    HjuArr = np.zeros([step_distance, steps_angle])
    print(steps_distance)
    print(steps_angle)
    cx = len(pixel_array)
    print(cx)
    ax = steps_distance * steps_angle * cx
    bx = 0
    print(ax)
    for d in range(0, steps_distance):
        for a in range(0, steps_angle):
            for p in pixel_array:
                if IsInLine(a, d, p.x, p.y):
                    HjuArr[d, a] += p.value
            bx += cx
            print("{0} / {1} : {2}%".format(bx, ax, bx/ax))
    return HjuArr

def HjuTransform2(step_angle, step_distance, steps_distance, steps_angle, pixel_array):
    HjuArr = np.zeros([steps_distance, steps_angle])
    print(steps_distance)
    print(steps_angle)
    size = np.shape(pixel_array)
    print(size)
    ax = steps_distance * steps_angle * size[0]*size[1]
    bx = 0
    print(ax)
    max_val = 0
    for d in range(2, steps_distance):
        for a in range(0, steps_angle):
            for x in range(0, size[0]):
                for y in range(0, size[1]):
                    if IsInLine(a*step_angle, d*step_distance, x, y):
                        HjuArr[d, a] += 1
            bx += size[0]*size[1]
            #if HjuArr[d, a] > max_val:
                #max_val = HjuArr[d,a]
            #print("{0} / {1} : {2}%".format(bx, ax, bx/ax))
    print("Transform done")
    #HjuArr = HjuArr / max_val
    return HjuArr

# Spektra aprēkins
def HjuTransform3(step_angle, step_distance, steps_distance, steps_angle, pixel_array):
    HjuArr = np.zeros([steps_angle, steps_distance])
    print(steps_distance)
    print(steps_angle)
    size = np.shape(pixel_array)
    print(size)
    ax = steps_distance * steps_angle * size[0]*size[1]
    bx = 0
    print(ax)
    for x in range(0, size[0]):
        for y in range(0, size[1]):
            value = pixel_array[x, y]
            if value < 1 :
                for a in range(0, steps_angle):
                    hju_val = (int)(x * math.cos(a*step_angle) + y * math.sin(a*step_angle))
                    if hju_val < steps_distance and hju_val > 0:
                        HjuArr[a, hju_val] += 1
    print("Transform done")
    return HjuArr


def HjuSpectrum(step_angle, step_distance, steps_distance, steps_angle, H_T):
    HjuSpec = np.zeros([steps_angle])
    max_val = 0
    for ai in range(0,steps_angle):
        for di in range(0,steps_distance):
            HjuSpec[ai] += (H_T[ai, di])*(H_T[ai, di])
        if HjuSpec[ai] > max_val:
                max_val = HjuSpec[ai]
    HjuSpec = HjuSpec / max_val
    print("Spectrum done")
    return HjuSpec


def HjuCircularCorrelation(step_angle, steps_angle, H_S_Map1, H_S_Map2):
    CC = np.zeros([steps_angle])
    for k in range(0, steps_angle):
        for i in range(0, steps_angle):
            CC[k] += (H_S_Map1[i])*(H_S_Map2[(i+k)//steps_angle])
        #if HjuSpec[ai] > max_val:
                #max_val = HjuSpec[ai]
    #HjuSpec = HjuSpec / max_val
    print("Spectrum done")
    return CC


def IsInLine(angle, distance, x, y, discretion_coef = 0.5):
    isLinePoint = (((x * math.cos(angle) + y * math.sin(angle)) >= (distance - discretion_coef)) and ((x * math.cos(angle) + y * math.sin(angle)) <= (distance + discretion_coef)))
    return isLinePoint


def X_Spectrum(data):
    size = np.shape(data)
    XSpec = np.zeros(size[0])
    max_val = 0
    for x in range(0,size[0]):
        for y in range(0,size[1]):
            if data[x,y] < 1:
                XSpec[x] += 1
    print("XSpectrum done")
    return XSpec


def Y_Spectrum(data):
    size = np.shape(data)
    YSpec = np.zeros(size[1])
    max_val = 0
    for x in range(0, size[0]):
        for y in range(0, size[1]):
            if data[x,y] < 1:
                YSpec[y] += 1
    print("YSpectrum done")
    return YSpec

def HjuLinearCorrelation(step_distance, steps_distance, X_spectrum_Map1, X_spectrum_Map2):
    LC = np.zeros([steps_distance])
    for t in range(0, steps_distance):
        for k in range(0, steps_distance):
            if t+k < steps_distance and t+k >= 0:
                LC[k] += (X_spectrum_Map1[t])*(X_spectrum_Map2[t+k])
    print("Spectrum done")
    return LC

def Rotate(matrix_, angle):
    rot = np.array([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]])
    print(rot)
    size = np.shape(matrix_)
    diag = int(math.sqrt(size[0]*size[0]+size[1]*size[1]))+1
    new_size = (2*diag, 2*diag)
    result = np.ones([new_size[0], new_size[1]])
    for x in range(0, size[0]):
        for y in range(0, size[1]):
            if matrix_[x,y] < 1:
                p = np.array([[x],[y]])
                result_p = np.matmul(rot, p)
                result[int(result_p[[0],[0]]), int(result_p[[1],[0]])] = 0
    return result


def GetLocalMaximaArgs(array_, number_of_maxima, peak_width):
    results = list()
    for i in range(0,len(array_)):
        if i>0:
            if i < len(array_)-1:
                if array_[i]>array_[i-1] and array_[i]>array_[i+1]:
                    results.append((i,array_[i]))
            else:
                if array_[i]>array_[i-1]:
                    results.append((i,array_[i]))
        else:
            if array_[i]>array_[i+1]:
                results.append((i,array_[i]))
    results.sort(reverse=True, key=lambda tup: tup[1])
    print("Sorted results: ")
    print(results)
    results_ret = np.array(results[:number_of_maxima])
    return results_ret


def GetArgNmax(array_, n):
    loc_array = np.copy(array_)
    loc_array = np.sort(loc_array)
    return loc_array[n]

def max(arg1, arg2):
    if arg1 >= arg2:
        return arg1
    else:
        return arg2

def min(arg1, arg2):
    if arg1 <= arg2:
        return arg1
    else:
        return arg2
