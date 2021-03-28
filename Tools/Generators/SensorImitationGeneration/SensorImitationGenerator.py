import pygame
import math

PI = 3.14159

class Line:
    def __init__(self, p1, p2):
        self.x1 = int(p1[0])
        self.x2 = int(p2[0])
        self.y1 = int(p1[1])
        self.y2 = int(p2[1])

    def minX(self):
        return min(self.x1, self.x2)

    def maxX(self):
        return max(self.x1, self.x2)

    def minY(self):
        return min(self.y1, self.y2)

    def maxY(self):
        return max(self.y1, self.y2)


class Object:
    def __init__(self, p1, a, id, orig):
        self.x1 = float(p1[0]-orig[0])
        self.y1 = float(p1[1]-orig[1])
        self.a = float(a)
        self.id = int(id)

    def distance(self, p2):
        return math.sqrt((self.x1-p2[0])*(self.x1-p2[0])+(self.y1-p2[1])*(self.y1-p2[1]))

    def angle(self, p2):
            return math.atan2(-(self.y1-p2[1]),-(self.x1-p2[0]))


def main():
    pygame.init()
    # load and set the logo
    logo = pygame.image.load("Icon.png")
    pygame.display.set_icon(logo)
    pygame.display.set_caption("Sensor data generator")

    # create a surface on screen that has the size of 240 x 180
    screen = pygame.display.set_mode((800, 400))
    envmap = pygame.image.load("SlamBackground.png")

    screen.blit(envmap, (150, 150))
    pygame.display.flip()

    fileString = str()
    fileNameOrig = 'SensorData'
    setInd = 1
    fileExt = '.dat'
    fileName = fileNameOrig + str(setInd) + fileExt
    template_see = "(see {0:.2f} ({1} {2:.2f} {3:.2f} {4:.2f} {5:.2f}))\n"
    time = 0.0
    obj_id = 1
    obj_distance = 2.2
    obj_angle = 3.3
    obj_speed = 4.4
    obj_ang_vel = 5.5
    template_move = "(move ({0:.2f} {1:.2f} {2:.2f})\n"
    moved_x = 0.0
    moved_y = 0.0
    moved_a = 0.0

    pos_x = 0.0
    pos_y = 0.0
    pos_a = 0.0

    orig_x = 200.0
    orig_y = 50.0
    orig_a = 0.0

    object_dict = dict()
    # create object dict
    for i in range (80, 480, 80):
        object = Object((i, 20), 0, i/80,(orig_x,orig_y))
        object_dict[object.id] = object
    # create environment
    linecolor = pygame.Color(40, 10, 100)
    linecolor2 = pygame.Color(30, 40, 20)
    for o in object_dict:
        pygame.draw.line(envmap, linecolor, (object_dict[o].x1+orig_x, object_dict[o].y1+20+orig_y), (object_dict[o].x1+orig_x, object_dict[o].y1-20+orig_y))
    pygame.draw.line(envmap, linecolor2, (0, 50), (500, 50))
    #pygame.draw.line(envmap, linecolor2, (0, 150), (500, 50))
    screen.blit(envmap, (150, 150))
    pygame.display.flip()

    running = True
    time = pygame.time.get_ticks()
    while running:
        # event handling, gets all event from the event queue
        for event in pygame.event.get():
            # only do something if the event is of type QUIT
            if event.type == pygame.MOUSEBUTTONDOWN:
                # move line
                moved_x = pygame.mouse.get_pos()[0] - orig_x - pos_x - 150
                moved_y = pygame.mouse.get_pos()[1] - orig_y - pos_y - 150
                moved_a = 0
                pos_x = pos_x + moved_x
                pos_y = pos_y + moved_y
                # calculate see
                time = pygame.time.get_ticks()
                obj_speed = 4.4
                obj_ang_vel = 5.5
                fileString = fileString + template_move.format(moved_x, moved_y, moved_a)
                for o in object_dict:
                    obj_id = object_dict[o].id
                    obj_distance = object_dict[o].distance((pos_x,pos_y))
                    obj_angle = object_dict[o].angle((pos_x,pos_y))
                    fileString = fileString + template_see.format(time, obj_id, obj_distance, obj_angle, obj_speed, obj_ang_vel)
                #print("OX: " + str(moved_x) + " OY: " + str(moved_y) + "A: " + str(obj_angle) + " OY: " + str(obj_distance));


                #print("Atan values")
                #print(math.atan2(1,1))
                #print(math.atan2(1,-1))
                #print(math.atan2(-1,1))
                #print(math.atan2(-1,-1))
                print("Coords")
                print("X: " + str(pygame.mouse.get_pos()[0]) + " Y: " + str(pygame.mouse.get_pos()[1]))
                print("MX: " + str(moved_x) + " Y: " + str(moved_y))


            if event.type == pygame.QUIT:
                # change the value to False, to exit the main loop
                try:
                    file = open(fileName, 'w')
                    file.write(fileString)
                    fileString = ''
                except FileNotFoundError:
                    print("Could not open file")
                running = False






if __name__ == '__main__':
    main()
