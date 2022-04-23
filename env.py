import numpy as np
import math
import pygame
from pygame import K_RETURN, K_RIGHT, K_LEFT, K_UP, K_DOWN, KEYUP
from global_vars import*
from pygame.math import Vector2
from algorithm import euc_dist
import numpy as np
from numpy import random


def drawGrid(WINDOW_WIDTH,WINDOW_HEIGHT,screen):
 #Set the size of the grid block
    field = np.zeros((WINDOW_WIDTH//blockSize,WINDOW_HEIGHT//blockSize))
    for x in range(0, WINDOW_WIDTH, blockSize):
        for y in range(0, WINDOW_HEIGHT, blockSize):
            rect = pygame.Rect(x, y, blockSize, blockSize)
            pygame.draw.rect(screen, BLACK, rect,1)
    return field

def draw_path(screen,path):
    for (i,j,k) in path:
        rect = pygame.Rect(i*5, j*5, 5, 5)
        pygame.draw.rect(screen, BLUE, rect)

def draw_PRM_path(screen,path,points):
    for (i,j) in path:
        pygame.draw.circle(screen,BLUE,(i,j),4)
    for p in points:
        p  = tuple(list(p))
        pygame.draw.circle(screen,BLACK,p,2)
    pygame.draw.lines(screen, GREEN,closed=False, points = path, width=1)

def make_black(screen,nodes):
    for node in nodes:
        for obs in node:
            rect = pygame.Rect(obs[0]*blockSize, obs[1]*blockSize, blockSize, blockSize)
            pygame.draw.rect(screen, BLACK, rect)

# def spread_fire(burning,burning_dict,obs_map,count):
#     additions = []
#     for key in burning_dict:
#         value = burning_dict[key]
#         if value-count==-400:
#             for obs in key:
#                 # goal = (obs[0]*3,obs[1])
#                 for k in obs_map:
#                     for node in k:
#                         if euc_dist(obs,node)<2:
#                             burning.append(k)
#                             additions.append(tuple(k))
#                             break
#     # for i in additions:
#     #     burning_dict[i] = count

#     return burning,burning_dict

def spread_fire(burning,obs_map):
    additions = []
    for obs in burning:
        for node in obs:
            for k in obs_map:
                for points in k:
                    if euc_dist(points,node)<2:
                        additions.append(k)
                        break
    burning = burning + additions
    for i in additions:
        if i in obs_map:
            obs_map.remove(i)
    return burning,obs_map


def check_to_extinguish(burning,curr_node,extinguish):
    for node in burning:
        for obs in node:
            goal = (obs[0]*3,obs[1]*3)
            if euc_dist(curr_node,goal)<5:
                extinguish.append(node)
                burning.remove(node)
                # print("watch",burning_dict)
                # del burning_dict[tuple(node)]
                break
    return burning,extinguish


def check_to_extinguishPRM(burning,curr_node,extinguish):
    for node in burning:
        for obs in node:
            goal = (obs[0]*15,obs[1]*15)
            if euc_dist(curr_node,goal)<25:
                extinguish.append(node)
                burning.remove(node)
                # print("watch",burning_dict)
                # del burning_dict[tuple(node)]
                break
    return burning,extinguish

def find_nearest(burning,curr_node):
    min_dist = 1000000
    out = (0,0)
    for node in burning:
        for obs in node:
            goal = (obs[0]*3,obs[1]*3)
            d  = euc_dist(curr_node,goal)
            if d<min_dist:
                min_dist = d
                out = obs         
    return out

def burn(screen,nodes):
    for node in nodes:
        for obs in node:
            rect = pygame.Rect(obs[0]*blockSize, obs[1]*blockSize, blockSize, blockSize)
            pygame.draw.rect(screen, RED, rect)

def burn_random(screen,obs_map):
    [rand_index] = np.random.choice(np.arange(0,len(obs_map)),1)
    for obs in obs_map[rand_index]:
        rect = pygame.Rect(obs[0]*blockSize, obs[1]*blockSize, blockSize, blockSize)
        pygame.draw.rect(screen, RED, rect)
    return obs_map[rand_index]
    

def generate_obstacles(screen,field):
    for (x, y), w in np.ndenumerate(field):
        color = GREEN if w == 1 else WHITE
        # size = np.sqrt(abs(w) / max_weight)
        size = blockSize
        rect = pygame.Rect(x*blockSize, y*blockSize, size, size)
        pygame.draw.rect(screen, color, rect)

def compute_obsmap(field,p):
    b = 0
    obs_map = []
    grid = WINDOW_WIDTH//blockSize
    T = grid*grid
    field = np.zeros((grid,grid))
    
    while((b/T)<p):
        [r] = np.random.choice(np.arange(1,5),1)
        [x] = np.random.choice(np.arange(2,grid),1)
        [y]  = np.random.choice(np.arange(2,grid-3),1)
        if(r==1):
            if(check_t1(field,x,y)):
                b+=4
                field,obs = place_t1(field,x,y)
                obs_map.append(obs)
            else:
                continue
        if(r==2):
            if(check_t2(field,x,y)):
                b+=4
                field,obs = place_t2(field,x,y)
                obs_map.append(obs)
            else:
                continue
        if(r==3):
            if(check_t3(field,x,y)):
                b+=4
                field,obs = place_t3(field,x,y)
                obs_map.append(obs)
            else:
                continue
        if(r==4):
            if(check_t4(field,x,y)):
                b+=4
                field,obs = place_t4(field,x,y)
                obs_map.append(obs)
            else:
                continue
    return field,obs_map


def check_t1(field,x,y):
        if(field[x][y]==0 and field[x][y+1]==0 and field[x][y+2]==0 and field[x][y+3]==0):
            return True
        else:
            return False
    
def check_t2(field,x,y):
    if(field[x][y]==0 and field[x][y+1]==0 and field[x][y+2]==0 and field[x-1][y+2]==0):
        return True
    else:
        return False

def check_t3(field,x,y):
        if(field[x][y]==0 and field[x][y+1]==0 and field[x-1][y+1]==0 and field[x-1][y+2]==0):
            return True
        else:
            return False

def check_t4(field,x,y):
        if(field[x][y]==0 and field[x][y+1]==0 and field[x][y+2]==0 and field[x-1][y+1]==0):
            return True
        else:
            return False

def place_t1(field,x,y):
    field[x][y] = 1
    field[x][y+1] = 1
    field[x][y+2] = 1
    field[x][y+3] = 1
    obs_index = [(x,y),(x,y+1),(x,y+2),(x,y+3)] 
    return field,obs_index

def place_t2(field,x,y):
    field[x][y] = 1
    field[x][y+1] = 1
    field[x][y+2] = 1
    field[x-1][y+2] = 1
    obs_index = [(x,y),(x,y+1),(x,y+2),(x-1,y+2)]
    return field,obs_index

def place_t3(field,x,y):
    field[x][y] = 1
    field[x][y+1] = 1
    field[x-1][y+1] = 1
    field[x-1][y+2] = 1
    obs_index = [(x,y),(x,y+1),(x-1,y+1),(x-1,y+2)]
    return field,obs_index

def place_t4(field,x,y):
    field[x][y] = 1
    field[x][y+1] = 1
    field[x][y+2] = 1
    field[x-1][y+1] = 1
    obs_index = [(x,y),(x,y+1),(x,y+2),(x-1,y+1)]
    return field,obs_index

class Player(pygame.sprite.Sprite):
    def __init__(self,pos=(80,40)):
        super(Player, self).__init__()
        self.surf = pygame.Surface((25, 11))
        self.surf.fill((0, 0, 255))
        self.surf.set_colorkey(BLACK)  
        self.image = self.surf.copy()
        self.image.set_colorkey(BLACK)
        self.rect = self.image.get_rect()
        pygame.draw.circle(self.surf, RED, (self.rect.midright),4)
        self.rect.center = pos
        self.new_image = self.surf.copy()
        self.position = Vector2(pos)
        self.direction = Vector2(1, 0)  # A unit vector pointing rightward.
        self.speed = 0.01
        self.angle_speed = 0
        self.angle = 0
        self.i = 1
    
   
    def navigate(self,path,curr_node,t):
        blockSize = 5
        if t<len(path) and (euc_dist(path[t],path[-1])>2.5):
            ind = t
            # m = (path[ind][1]-path[ind-1][1])/(path[ind][0]-path[ind-1][0])
            # if ind == len(path)-1:
            #     ang = 0
            # else:
            ang = round(path[ind][2],2)
            # print("ang",ang)
            self.angle = round((ang*180/math.pi))
            # print(path[ind][2],self.angle)
            self.new_image = pygame.transform.rotate(self.surf, -self.angle)
            self.rect = self.new_image.get_rect()
            self.rect.centerx = (path[ind][0]*blockSize)
            self.rect.centery = (path[ind][1]*blockSize)
            reached = False
            curr_node = path[ind]
        else:
            reached = True
        return reached,curr_node

    def PRM_navigate(self,path,curr_node,t):
        blockSize = 1
        if t<len(path) and (euc_dist(path[t],path[-1])>2.5):
            ind = t
            # m = (path[ind][1]-path[ind-1][1])/(path[ind][0]-path[ind-1][0])
            # if ind == len(path)-1:
            # ang = 0
            # else:
            # ang = round(path[ind][2],2)
            ang = math.atan2((path[ind][1]-path[ind-1][1]),(path[ind][0]-path[ind-1][0]))
            # print("ang",ang)
            self.angle = round((ang*180/math.pi))
            # print(path[ind][2],self.angle)
            self.new_image = pygame.transform.rotate(self.surf, -self.angle)
            self.rect = self.new_image.get_rect()
            self.rect.centerx = (path[ind][0]*blockSize)
            self.rect.centery = (path[ind][1]*blockSize)
            reached = False
            curr_node = path[ind]
        else:
            reached = True
        return reached,curr_node



class Dynamic_obs(pygame.sprite.Sprite):
    def __init__(self):
        super(Dynamic_obs, self).__init__()
        self.surf = pygame.Surface((50, 25))
        self.surf.fill(BLACK)
        self.rect = self.surf.get_rect(
            center=(
                random.randint(5, 500),
                random.randint(5, 500),
            )
        )
        self.speed = 1

    # Move the sprite based on speed
    # Remove the sprite when it passes the left edge of the screen
    def update(self):
        self.rect.move_ip(self.speed, 0)
        if self.rect.right < 0:
            self.kill()