from numpy import angle, append
import pygame
from env import Dynamic_obs, burn_random, check_to_extinguishPRM, draw_PRM_path, draw_path, drawGrid, find_nearest, generate_obstacles, compute_obsmap, burn, make_black, spread_fire
from env import Player
from global_vars import*
from PRM import*
import matplotlib.pyplot as plt
import numpy as np
prm_time = 0
pygame.init()



# Fill the background with white
screen = pygame.display.set_mode([WINDOW_WIDTH,WINDOW_HEIGHT])

player = Player((15,15))

field = drawGrid(WINDOW_WIDTH,WINDOW_HEIGHT,screen)


running = True
count=0

field,obstacle_map = compute_obsmap(field,0.02)
initial_obs = len(obstacle_map)
obs_map = obstacle_map
print('yaaaaay')
generate_obstacles(screen,field)
print("look")
# screen.fill(WHITE)

moving_obs = pygame.sprite.Group()
new_obs = Dynamic_obs()
new_obs1 = Dynamic_obs()
new_obs2 = Dynamic_obs()
moving_obs.add(new_obs)
moving_obs.add(new_obs1)
moving_obs.add(new_obs2)

screen.blit(player.new_image,player.rect)
pygame.display.flip()

start = (1,1)
curr_node = start
goal = (550,550)
prm = PRMController(numOfRandomCoordinates=2000, allObs=obs_map, current=start,destination=goal)
path,points = prm.runPRM(initialRandomSeed=0,screen = screen)
# print("goal:",goal)


draw_PRM_path(screen,path,points)
while running:
    count+=1    
    # screen1 = pygame.display.set_mode([WINDOW_WIDTH,WINDOW_HEIGHT])
    # print("see",burning_dict)
    screen.fill(WHITE)
    generate_obstacles(screen,field)
    draw_PRM_path(screen,path,points)

    reached,curr_node = player.PRM_navigate(path,curr_node,player.i)
    # print(burning,extinguish)
    if reached:
       running = False
    #
    # Did the user click the window close button?
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            # If the Esc key is pressed, then exit the main loop
            if event.key == pygame.K_ESCAPE:
                running = False
          
        # Check for QUIT event. If QUIT, then set running to false.
    if count%20==0:
        player.i+=1
        for obs in moving_obs:
            Dynamic_obs.update(obs)
    for obs in moving_obs:
        screen.blit(obs.surf,obs.rect)    

    screen.blit(player.new_image,player.rect)
    # Flip the display
    pygame.display.flip()



# Done! Time to quit.
pygame.quit()