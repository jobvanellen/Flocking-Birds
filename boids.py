import numpy as np
import random
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import copy

#from matplotlib import animation

# position updates: new_pos = old_pos + speed*dt
# speed updates: new_speed = old_speed + accel*dt

# use deepcopy for speed&pos update -> own copy func
# minimum speed, halting midair seems impossible

# next steps:
# 1: add limited field of view. V
# 2: obstacles
# 3: goal V
# 4: disperse/scatter V


# optional: turn rules on and off at runtime

## simulation settings
# size of field
field_size = 200

num_boids = 50
time = 500.0 # seconds, total simulation time
dt = .05 # seconds, simulation timestep
plot_interval = 1.0 # seconds

boids = []
boids_old = []

goal = np.ones(3)*field_size/2
print(goal)

cohesion_area = 50
alignment_area = 25
separation_area = 5

start_speed = 20

disperse = False # allow for scattering behaviour if true

position = np.random.uniform(0.0, field_size,(num_boids, 3))
velocity = np.random.uniform(start_speed*-1, start_speed, (num_boids,3))

position_old = copy.deepcopy(position)
velocity_old = copy.deepcopy(velocity)

# setting up plot stuff
plt.ion()
fig = plt.figure()
ax = fig.gca(projection='3d')

## core functions
# update visual
def draw_boids(t):
    ax.clear()
    ax.set(xlim=(-10,field_size+10), ylim=(-10,field_size+10), zlim=(-10,field_size+10))
    #ax.scatter(goal[0], goal[1], goal[2], color="green")
    for b in range (num_boids):
        ax.quiver(position[b,0], position[b,1], position[b,2],\
            velocity[b,0],velocity[b,1],velocity[b,2], length=4, normalize=True)
    timestring = str(t)
    ax.text(0,0,0, timestring)
    plt.show(block = False)
    plt.pause(0.0001)


# calculate new boid positions based on ruleset
def move_boids():
    #vector multipliers
    m1 = 1.0 # cohesion
    m2 = 1.0 # separation
    m3 = 2.0 # alignment
    m4 = 1.0 # bound position
    m5 = 1.0 # flock to goal

    if disperse: # if disperse flip m1
        m1 = m1*-1

    for b in range(num_boids):
        v1 = m1 * cohesion(b)
        v2 = m2 * separation(b)
        v3 = m3 * alignment(b)
        v4 = m4 * bound_position(b)
        v5 = m5 * flock_to_goal(b)


        velocity[b] = velocity[b] + (v1 + v2 + v3 + v4)*dt
        velocity[b] = limit_velocity(b)
        position[b] = position[b] + velocity[b]*dt

    copy_boids()

## basic ruleset
# boids try to fly to the centre of mass of neighbouring boids
# metre/second/pos_diff
def cohesion(current_boid):
    v = np.zeros(3)
    num = 0
    for b in boids_old:
        if in_range(current_boid, b, cohesion_area):
            v = v + position[b] - position[current_boid]
            num = num + 1
    
    if num > 0:
        v = v/num

    return v/100

# boids try not to collide with other boids
def separation(current_boid):
    v = np.zeros(3)   
    for b in boids_old:
        if in_range(current_boid, b, separation_area):
            v = v - (position[b] - position[current_boid])
    return v

# boids try to match velocity with near boids
def alignment(current_boid):
    v = np.zeros(3)
    num = 0    
    for b in boids_old:
        if in_range(current_boid, b, alignment_area):
            v = v + velocity[b]
            num = num + 1

    if num > 0:
        v = v/num

    return (v - velocity[current_boid])/8

## extended ruleset
# change course to stay inside world
def bound_position(current_boid):
    v = np.zeros(3)
    factor = 10 # steering influence level

    if position[current_boid,0] < 0.0:
        v[0] = factor
    elif position[current_boid,0] > field_size:
        v[0] = -1 * factor

    if position[current_boid,1] < 0.0:
        v[1] = factor
    elif position[current_boid,1] > field_size:
        v[1] = -1 * factor

    if position[current_boid,2] < 0.0:
        v[2] = factor
    elif position[current_boid,2] > field_size:
        v[2] = -1 * factor

    return v

def flock_to_goal(current_boid):
    return (goal - position[current_boid]) / 100


## helper functions
# checks if boid is within range r of other boid
def in_range(b1, b2, r):
    #check self
    if b1 == b2:
        return False
    if abs(position[b1,0]-position_old[b2,0]) > r:
        return False
    if abs(position[b1,1]-position_old[b2,1]) > r:
        return False
    if abs(position[b1,2]-position_old[b2,2]) > r:
        return False
    return True

def limit_velocity(b):
    vlim = 10
    v = velocity[b]
    
    for i in range(0,3):
        if abs(velocity[b,i]) > vlim:
            v[i] = (velocity[b,i] / abs(velocity[b,i])) * vlim 
    return v

def copy_boids():
    global boids_old
    boids_old = []
    for b in range (num_boids):
        boids_old.append(b)


## main
def main():
    #init_boids()
    t = 0.0
    global disperse
    while t < time:
        t = round(t, 3) # round t for ease of use
        #print(boids_old==boids)
        if 100.0 <= t < 150.0:
            disperse = True
        move_boids()
        # plot every second
        if t % plot_interval == 0.0:
            draw_boids(t)
        t = t+dt

if __name__ == "__main__":
    main()


# %%
