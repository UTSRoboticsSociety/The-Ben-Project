import pygame

#Written by Fedor


def draw_point(screen, x, y):
    pygame.draw.circle(screen, (255, 140, 0), (int(x), int(y)), 6)

def draw_line(screen, x1, y1, x2, y2):
    pygame.draw.line(screen, (255, 140, 0), (int(x1), int(y1)), (int(x2), int(y2)), 3)

def draw(screen, s): #will need to map world coordinates to image plane coordinates for accurate tracking

    CENTER_X = 400
    CENTER_Y = 190
    N = 1

    for i in s:
        point = [(i[1] + CENTER_X) * N, (i[2] + CENTER_Y) * N]
        if i[0] == "HEAD":
            HEAD = point
        elif i[0] == "SPINE_SHOULDER":
            SPINE_SHOULDER = point
        elif i[0] == "SPINE_BASE":
            SPINE_BASE = point
        elif i[0] == "WRIST_RIGHT":
            WRIST_RIGHT = point
        elif i[0] == "ELBOW_RIGHT":
            ELBOW_RIGHT = point
        elif i[0] == "WRIST_LEFT":
            WRIST_LEFT = point
        elif i[0] == "ELBOW_LEFT":
            ELBOW_LEFT = point
        elif i[0] == "KNEE_RIGHT":
            KNEE_RIGHT = point
        elif i[0] == "ANKLE_RIGHT":
            ANKLE_RIGHT = point
        elif i[0] == "KNEE_LEFT":
            KNEE_LEFT = point
        elif i[0] == "ANKLE_LEFT":
            ANKLE_LEFT = point
        draw_point(screen, point[0], point[1])
    
    draw_line(screen, HEAD[0], HEAD[1], SPINE_SHOULDER[0], SPINE_SHOULDER[1])
    draw_line(screen, SPINE_SHOULDER[0], SPINE_SHOULDER[1], SPINE_BASE[0], SPINE_BASE[1])
    draw_line(screen, SPINE_SHOULDER[0], SPINE_SHOULDER[1], ELBOW_RIGHT[0], ELBOW_RIGHT[1])
    draw_line(screen, ELBOW_RIGHT[0], ELBOW_RIGHT[1], WRIST_RIGHT[0], WRIST_RIGHT[1])
    draw_line(screen, SPINE_SHOULDER[0], SPINE_SHOULDER[1], ELBOW_LEFT[0], ELBOW_LEFT[1])
    draw_line(screen, ELBOW_LEFT[0], ELBOW_LEFT[1], WRIST_LEFT[0], WRIST_LEFT[1])
    draw_line(screen, SPINE_BASE[0], SPINE_BASE[1], KNEE_RIGHT[0], KNEE_RIGHT[1])
    draw_line(screen, KNEE_RIGHT[0], KNEE_RIGHT[1], ANKLE_RIGHT[0], ANKLE_RIGHT[1])
    draw_line(screen, SPINE_BASE[0], SPINE_BASE[1], KNEE_LEFT[0], KNEE_LEFT[1])
    draw_line(screen, KNEE_LEFT[0], KNEE_LEFT[1], ANKLE_LEFT[0], ANKLE_LEFT[1])
