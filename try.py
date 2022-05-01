import sys, pygame
from pygame.locals import*

width = 1000
height = 500
screen_color = (49, 150, 100)
line_color = (255, 0, 0)

def main():
    screen=pygame.display.set_mode((width,height))
    # screen.fill(screen_color)
   
    pygame.draw.line(screen,line_color, (60, 80), (130, 100))
    pygame.display.flip()
    while True:
        for events in pygame.event.get():
            if events.type == QUIT:
                sys.exit(0)
main()