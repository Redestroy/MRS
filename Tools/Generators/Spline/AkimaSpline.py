import pygame
import random


def get_p3(index, arg_val, func_val, slope_val):
    return (3*(func_val[index + 1]-func_val[index])/(arg_val[index+1]-arg_val[index])-2*slope_val[index]-slope_val[index+1])/(arg_val[index + 1] - arg_val[index])


def get_p4(index, arg_val, func_val, slope_val):
    return (slope_val[index]+slope_val[index+1]-2*(func_val[index + 1]-func_val[index])/(arg_val[index+1]-arg_val[index]))/(arg_val[index + 1] - arg_val[index])/(arg_val[index + 1] - arg_val[index])


def poly_value(arg, argi, poly):
    return poly[0] + poly[1]*(arg-argi) + poly[2]*(arg-argi)^2 + poly[3]*(arg-argi)^3

def get_m(index, arg_val, func_val):
    if index == -2:
        m_n1 = get_m(-1, arg_val, func_val)
        return 2*m_n1 - get_m(0, arg_val, func_val)
    elif index == -1:
        return 2*get_m(0, arg_val, func_val) - get_m(1, arg_val, func_val)
        pass
    elif index == len(arg_val):
        return 2*get_m(len(arg_val) - 1, arg_val, func_val) - 2*get_m(len(arg_val) - 2, arg_val, func_val)
        pass
    elif index == len(arg_val) + 1:
        return 2*get_m(len(arg_val), arg_val, func_val) - 2*get_m(len(arg_val) - 1, arg_val, func_val)
    elif index == len(arg_val) + 2:
        return 0
    else:
        dx = arg_val[index+2+1]-arg_val[index+2]
        dy = func_val[index+2+1]-func_val[index + 2]
        return dy/dx


def main():
    pygame.init()
    # load and set the logo
    logo = pygame.image.load("Icon.png")
    pygame.display.set_icon(logo)
    pygame.display.set_caption("Planner")

    # create a surface on screen that has the size of 240 x 180
    screen = pygame.display.set_mode((800, 800))
    envmap = pygame.image.load("Environment500.png")
    tstart = 0
    tend = 10
    deltaTime = 0.2
    t_val = list()
    x_val = list()
    y_val = list()
    z_val = list()
    t = tstart
    while t < tend:
        t_val.append(t)
        t += deltaTime
    for i in range(0, len(t_val) + 10):
        x_val.append(random.randint(0,400))
        y_val.append(random.randint(0,400))
        z_val.append(random.randint(0,20))
    sx_val = list()
    sy_val = list()
    sz_val = list()
    mx_val = list()
    my_val = list()
    mz_val = list()
    for i in range(0, len(t_val)):
        for j in range(-2, 2, 1):
            mx_val.insert(j+2, get_m(i+j, t_val, x_val))
            my_val.insert(j+2, get_m(i+j, t_val, y_val))
            mz_val.insert(j+2, get_m(i+j, t_val, z_val))
        if abs(mx_val[3]-mx_val[2])+abs(mx_val[1]-mx_val[0]) == 0:
            temp = (mx_val[2] + mx_val[1])/2
        else:
            temp = (abs(mx_val[3]-mx_val[2])*mx_val[1] + abs(mx_val[1]-mx_val[0])*mx_val[2])/(abs(mx_val[3]-mx_val[2])+abs(mx_val[1]-mx_val[0]))
        sx_val.insert(i, temp)
        if abs(my_val[3]-my_val[2])+abs(my_val[1]-my_val[0]) == 0:
            temp = (my_val[2] + my_val[1])/2
        else:
            temp = (abs(my_val[3]-my_val[2])*my_val[1] + abs(my_val[1]-my_val[0])*my_val[2])/(abs(my_val[3]-my_val[2])+abs(my_val[1]-my_val[0]))
        sy_val.insert(i, temp)
        if abs(mz_val[3]-mz_val[2])+abs(mz_val[1]-mz_val[0]) == 0:
            temp = (mz_val[2] + mz_val[1])/2
        else:
            temp = (abs(mz_val[3]-mz_val[2])*mz_val[1] + abs(mz_val[1]-mz_val[0])*mz_val[2])/(abs(mz_val[3]-mz_val[2])+abs(mz_val[1]-mz_val[0]))
        sz_val.insert(i,temp)
    poliesx = dict()
    poliesy = dict()
    poliesz = dict()
    polies = {
        'x': poliesx,
        'y': poliesy,
        'z': poliesz
    }
    for i in range(0, len(t_val)):
        poliesx[t_val] = {x_val[i], t_val[i], get_p3(i, t_val, x_val, sx_val), get_p4(i, t_val, x_val, sx_val)}
        poliesy[t_val] = {y_val[i], t_val[i], get_p3(i, t_val, y_val, sy_val), get_p4(i, t_val, y_val, sy_val)}
        poliesz[t_val] = {z_val[i], t_val[i], get_p3(i, t_val, z_val, sz_val), get_p4(i, t_val, z_val, sz_val)}

    rectcolor = pygame.Color(200, 200, 200)
    linecolor = pygame.Color(100, 200, 100)
    dotcolor = pygame.Color(200, 100, 100)
    for i in range(0,len(t_val)):
        # draw splines
        t = t_val[i];
        for arg in range (t_val, t_val+deltaTime - deltaTime/5, deltaTime/5):
            pygame.draw.line(envmap, linecolor, (poly_value(arg, t, poliesx[t]), poly_value(arg, t, poliesy[t])), ((poly_value(arg + deltaTime/5, t, poliesx[t]), poly_value(arg + deltaTime/5, t, poliesy[t]))))
        # draw main points
        pygame.draw.circle(envmap,dotcolor,(x_val[i], y_val[i]), 3)
    screen.blit(envmap, (150, 150))
    pygame.display.flip()
    # define a variable to control the main loop
    running = True

    # main loop
    while running:
        # event handling, gets all event from the event queue
        for event in pygame.event.get():
            # only do something if the event is of type QUIT
            if event.type == pygame.MOUSEBUTTONDOWN:
                print("Coords")
                print("X: " + str(pygame.mouse.get_pos()[0]) + " Y: " + str(pygame.mouse.get_pos()[1]))
            if event.type == pygame.QUIT:
                # change the value to False, to exit the main loop
                running = False

if __name__ == '__main__':
    main()
