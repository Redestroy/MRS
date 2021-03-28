import pygame

def GetParticlePos(input):
    pos = (parse_one, parse_second)
    return pos

def GetParticleData(input):
    data = dict()
    data['id'] = id
    data['pos'] = pos
    data['objects'] = list()
    return data

def GetParticleObjects(input): #returns dict
    input_list = input.split('(',')')
    objects = dict()
    for str in input_list:
        objects[str[2:4]] = str
    return objects

def main():
    # read sensor file
    path = 'InputFiles\\'
    name = 'SensorData1.dat'
    fileName = path + name
    inputstr = ''
    input_string_list = list()
    try:
        file = open(fileName, 'w')
        inputstr = file.read()
        input_string_list = inputstr.splitlines()
    except FileNotFoundError:
        print("Could not open file")
    # read output file
    path2 = 'InputFiles\\'
    name2 = 'ParticleResult.res'
    fileName2 = path2 + name2
    inputstr2 = ''
    input_string_list2 = list()
    try:
        file2 = open(fileName, 'w')
        inputstr2 = file2.read()
        input_string_list2 = inputstr2.splitlines()
    except FileNotFoundError:
        print("Could not open file")

    path3 = 'InputFiles\\'
    name3 = 'EntireParticlePath.res'
    fileName3 = path3 + name3
    inputstr3 = ''
    input_string_list3 = list()
    try:
        file3 = open(fileName, 'w')
        inputstr3 = file3.read()
        input_string_list3 = inputstr3.splitlines()
    except FileNotFoundError:
        print("Could not open file")

    # draw background
    # initialize the pygame module
    pygame.init()
    # load and set the logo
    logo = pygame.image.load("Icon.png")
    pygame.display.set_icon(logo)
    pygame.display.set_caption("Slam Visualizer")

    # create a surface on screen that has the size of 240 x 180
    screen = pygame.display.set_mode((800, 800))
    envmap = pygame.image.load("SlamBackground.png")
    # draw object lines
    # Get objects from sensor files

    # draw particle lines
    particles = dict()
    i = 1;
    for p in particles:
        robot_pos_color = pygame.Color(20*i, 200, 100)
        i = i + 1

    # Get objects from result files
    # draw robot position
    robot_pos_color = pygame.Color(100, 200, 100)

    # timelapse feature



if __name__ == "__main__":
    main()
