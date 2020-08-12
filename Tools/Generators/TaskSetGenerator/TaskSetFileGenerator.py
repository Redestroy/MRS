import random
import math


def generate_time(function_type, index, delta, offset, n_tasks):
    if function_type == 'I':
        return index * delta + offset
    elif function_type == 'D':
        return (n_tasks - index) * delta + offset
    elif function_type == 'S':
        return int(offset + index)
    elif function_type == 'C':
        return int(delta*(index//n_tasks) + offset + index)
    elif function_type == 'R':
        return random.randint(0, n_tasks*delta)
    else:
        return offset


def generate_x(function_type, index, pad, width, offset, n_tasks):
    if function_type == 'E':
        return (index % n_tasks)*(width - 2*pad)/(n_tasks-1) - offset + pad
    elif function_type == 'C':
        return math.cos(2*math.pi*(index % n_tasks)/n_tasks)*(width*index//n_tasks+pad)
    elif function_type == 'G':
        return math.cos(2*math.pi*(index % n_tasks)/n_tasks)*(width*index//n_tasks+pad) + offset
    elif function_type == 'M':
        return math.cos(2*math.pi*(index % n_tasks)/n_tasks)*(width*index % n_tasks+pad) + offset[index//n_tasks]
    elif function_type == 'R':
        return random.randint(pad, width-pad)-offset
    else:
        return index*pad


def generate_y(function_type, index, pad, width, offset, n_tasks, n_tasks2):
    if function_type == 'E':
        return (index//n_tasks)*(width - 2*pad)/(n_tasks2-1) - offset + pad
    elif function_type == 'C':
        return math.sin(2*math.pi*(index % n_tasks)/n_tasks)*(width*index//n_tasks+pad)
    elif function_type == 'G':
        return math.sin(2*math.pi*(index % n_tasks)/n_tasks)*(width*index//n_tasks+pad) + offset
    elif function_type == 'M':
        return math.sin(2*math.pi*(index % n_tasks)/n_tasks)*(width*index % n_tasks+pad) + offset[index//n_tasks]
    elif function_type == 'R':
        return random.randint(pad, width-pad)-offset
    else:
        return index*pad
    pass


def generate_weight(argument):
    return argument


def generate(argument, function_type, parameter, step, max_x, max_y, row, column, n_tasks):
    X_DISTANCE = step
    Y_DISTANCE = step * column
    if function_type == 'T':
        if parameter == 'D1':
            return argument * TIME_CONSTANT + 0
        elif parameter == 'D2':
            return n_tasks*TIME_CONSTANT - argument * TIME_CONSTANT + 0
        elif parameter == 'R':
            return argument * random.randint()
    if function_type == 'X':
        if parameter == 'D1':
            return (argument % (Y_DISTANCE // X_DISTANCE) + 1) * X_DISTANCE
        elif parameter == 'R':
            return argument * random.randint()
    if function_type == 'Y':
        if parameter == 'D1':
            return X_DISTANCE * (1 + argument * X_DISTANCE // Y_DISTANCE)
        elif parameter == 'R':
            return argument * random.randint()
    if function_type == 'W':
        if parameter == 'D1':
            return 1 + argument * DEFAULT_WEIGHT
        elif parameter == 'W2':
            return argument
        elif parameter == 'R':
            return argument * random.randint()


def constrain(val, min, max, pad):
    if val < min + pad:
        return min + pad
    elif val > max - pad:
        return max - pad
    else:
        return val


TIME_CONSTANT = 5
X_DISTANCE = 3
Y_DISTANCE = 300
DEFAULT_WEIGHT = 5


def main():

    max_x = 3000
    max_y = 3000
    offset_x = 1500
    offset_y = 1500
    step = 500
    rows = 3
    columns = 5
    distributions = {
        'E',
        'C',
        'G',
        'M',
        'R'
    }
    dispatches = {
        'I',
        'D',
        'S',
        'C',
        'R'
    }

    taskSets = list()

    for dt in distributions:
        for dp in dispatches:
            taskSets.append((dt, dp))


    n_tasks = 15
    delay = 30
    fileString = str();
    fileNameOrig = 'TaskSet'
    setInd = 1
    fileExt = '.dat'
    fileName = fileNameOrig + str(setInd) + fileExt
    template = "{0} T: {1} 0 T_A {2} /C_S C_P {3:.2f} {4:.2f} 0.00 5.00/C_E C_N/T_A 4 /A_L {5}/A_W {6}/A_L {7}/A_W {8}/\n"

    for ts in taskSets:
        fileName = fileNameOrig + str(setInd) + fileExt
        setInd += 1

        if ts[0] == 'E':
            width_x = max_x
            width_y = max_y
            offset_x_t = offset_x
            offset_y_t = offset_y
            n_tasks_c_x = rows
            n_tasks_c_y = n_tasks/rows
            pass
        elif ts[0] == 'C':
            width_x = 300
            width_y = 300
            offset_x_t = 0
            offset_y_t = 0
            n_tasks_c_x = 5
            n_tasks_c_y = 5
            pass
        elif ts[0] == 'G':
            width_x = 200
            width_y = 200
            offset_x_t = offset_x/3
            offset_y_t = offset_y/3
            n_tasks_c_x = 5
            n_tasks_c_y = 5
            pass
        elif ts[0] == 'M':
            width_x = 300
            width_y = 100
            offset_x_t = [offset_x/3, -offset_x/3, 0]
            offset_y_t = [offset_y/2, offset_y/2, -offset_y/2]
            n_tasks_c_y = 5
            n_tasks_c_x = 5
        elif ts[0] == 'R':
            width_x = max_x
            width_y = max_y
            offset_x_t = offset_x
            offset_y_t = offset_y
            n_tasks_c_y = 15
            n_tasks_c_x = 15
        else:
            width_x = max_x
            width_y = max_y
            offset_x_t = offset_x
            offset_y_t = offset_y
            n_tasks_c_y = 15
            n_tasks_c_x = 15
        # Create dispatch variables
        delta_time = 5
        offset_t = 1
        if ts[1] == 'I':
            n_tasks_t = n_tasks
        elif ts[1] == 'D':
            n_tasks_t = n_tasks
        elif ts[1] == 'S':
            n_tasks_t = n_tasks
        elif ts[1] == 'C':
            n_tasks_t = n_tasks/3
        elif ts[1] == 'R':
            n_tasks_t = n_tasks
        else:
            n_tasks_t = n_tasks

        for ind in range(0, n_tasks):
            t = constrain(generate_time(ts[1], ind, delta_time, offset_t, n_tasks_t), 0, 600, 0);
            x = constrain(generate_x(ts[0], ind, 300, width_x, offset_x_t, n_tasks_c_x), -offset_x, max_x-offset_x, 200)
            y = constrain(generate_y(ts[0], ind, 300, width_y, offset_y_t, n_tasks_c_x, n_tasks_c_y), -offset_y, max_y-offset_y, 200)
            w = generate_weight(10)
            d = delay
            fileString = fileString + template.format(t, ind, w, x, y, ind, d, ind, d)
        try:
            file = open(fileName, 'w')
            file.write(fileString)
            fileString = ''
        except FileNotFoundError:
            print("Could not open file")

if __name__ == '__main__':
    main()

