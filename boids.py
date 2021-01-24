import numpy as np
import random
from mpl_toolkits.mplot3d import Axes3D
import matplotlib
matplotlib.use('TkAgg') # speed up plotting
import matplotlib.pyplot as plt
import copy
import math

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
# 5: predator


# optional: turn rules on and off at runtime

## simulation settings
# size of field
field_size = 200

num_boids = 100
time = 150.0 # seconds, total simulation time
dt = 0.05 # seconds, simulation timestep
plot_interval = 0.1 # seconds
t = 0.0

target = np.ones(3)*field_size/2
#print(goal)

cohesion_area = 60
alignment_area = 30
separation_area = 6

predator_area = 20

start_speed = 50
vlim = 60
vmin = 10

disperse = False # allow for scattering behaviour if true

position = np.random.uniform(0.0, field_size,(num_boids, 3))
velocity = np.random.uniform(start_speed*-1, start_speed, (num_boids,3))


position_old = copy.deepcopy(position)
velocity_old = copy.deepcopy(velocity)

pos_predator = np.zeros((1,3))
vel_predator = np.ones((1,3)) * 55



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
    ax.quiver(pos_predator[0,0], pos_predator[0,1], pos_predator[0,2],\
        vel_predator[0,0], vel_predator[0,1], vel_predator[0,2], length=7, normalize = True, color = "red")
    timestring = str(t)
    ax.text(0,0,0, timestring)

    fname = 'Flock-%03.1f.png' % t
    plt.savefig('images/'+fname)

    plt.show(block = False)
    plt.pause(0.0001)


# calculate new boid positions based on ruleset
def move_boids():
    #vector multipliers
    m1 = 100.0 # cohesion
    m2 = 8.0 # separation
    m3 = 1.0 # alignment
    m4 = 20.0 # bound position
    m5 = 1.0 # flock to goal
    m6 = 200.0 # flee from predator

    if disperse: # if disperse flip m1
        m1 = m1*-1

    for b in range(num_boids):
        v1 = np.zeros(3)
        v2 = np.zeros(3)
        v3 = np.zeros(3)
        v4 = np.zeros(3)
        v5 = np.zeros(3)
        v6 = np.zeros(3)

        v1 = m1 * cohesion(b)
        v2 = m2 * separation(b)
        v3 = m3 * alignment(b)
        v4 = m4 * bound_position(b)
        if t < 25:
            v5 = m5 * flock_to_goal(b, target)
        v6 = m6 * flee_from_predator(b)


        velocity[b] = velocity_old[b] + (v1 + v2 + v3 + v4 + v6)*dt
        limit_velocity(b)
        position[b] = position_old[b] + velocity[b]*dt

    copy_boids()

## basic ruleset
# boids try to fly to the centre of mass of neighbouring boids
# metre/second/pos_diff
def cohesion(current_boid):
    v = np.zeros(3)
    num = 0
    for b in range(num_boids):
        if in_range(current_boid, b, cohesion_area):
            v = v + position_old[b] - position[current_boid]
            num = num + 1
    
    if num > 0:
        v = v/num

    return v/100

# boids try not to collide with other boids
def separation(current_boid):
    v = np.zeros(3)   
    for b in range(num_boids):
        if in_range(current_boid, b, separation_area):
            v = v - (position_old[b] - position_old[current_boid])
    return v

# boids try to match velocity with near boids
def alignment(current_boid):
    v = np.zeros(3)
    num = 0    
    for b in range(num_boids):
        if in_range(current_boid, b, alignment_area):
            v = v + velocity_old[b]
            num = num + 1
    if num > 0:
        v = v/num

    return v - velocity_old[current_boid]

## extended ruleset
# change course to stay inside world
def bound_position(current_boid):
    v = np.zeros(3)
    factor = 50 # steering influence level

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

def flock_to_goal(current_boid, goal):
    return (goal - position[current_boid]) / 100

def flee_from_predator(current_boid):
    v = np.zeros(3)
    if abs(position[current_boid,0]-pos_predator[0,0]) > predator_area:
        return v 
    if abs(position[current_boid,1]-pos_predator[0,1]) > predator_area:
        return v
    if abs(position[current_boid,2]-pos_predator[0,2]) > predator_area:
        return v
    v = -1 * (pos_predator[0] - position[current_boid])
    return v

## predator functions
def move_predator():
    v1 = -1*(find_nearest_boid()-position[0])*50
    v2 = 20 * bound_predator(0)
    vel_predator[0] = vel_predator[0] + (v1 + v2) * dt
    limit_predator_velocity()
    pos_predator [0] = pos_predator + vel_predator * dt

def find_nearest_boid():
    nearest_boid = np.ones(3)*field_size
    d_nearest = math.sqrt(field_size**2+field_size**2+field_size**2)
    for b in range(num_boids):
        d = math.sqrt((abs(position[b,0]-pos_predator[0,0])**2+abs(position[b,1]-pos_predator[0,1])**2+abs(position[b,2]-pos_predator[0,2])**2))
        if d < d_nearest:
            d_nearest = d
            nearest_boid = position[b]
    return nearest_boid

def bound_predator(p):
    v = np.zeros(3)
    factor = 10 # steering influence level

    if pos_predator[p,0] < 0.0:
        v[0] = factor
    elif pos_predator[p,0] > field_size:
        v[0] = -1 * factor

    if pos_predator[p,1] < 0.0:
        v[1] = factor
    elif pos_predator[p,1] > field_size:
        v[1] = -1 * factor

    if pos_predator[p,2] < 0.0:
        v[2] = factor
    elif pos_predator[p,2] > field_size:
        v[2] = -1 * factor

    return v

def limit_predator_velocity():
    for i in range(0,3):
        if abs(vel_predator[0,i]) > vlim:
            vel_predator[0,i] = (vel_predator[0,i] / abs(vel_predator[0,i])) * vlim




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
    for i in range(0,3):
        if abs(velocity[b,i]) > vlim:
            velocity[b,i] = (velocity[b,i] / abs(velocity[b,i])) * vlim
        if abs(velocity[b,i]) < vmin:
            velocity[b,i] = (velocity[b,i] / abs(velocity[b,i])) * vmin

def copy_boids():
    global position_old
    global velocity_old
    
    position_old = copy.deepcopy(position)
    velocity_old = copy.deepcopy(velocity)
    #position_old = np.array(ujson.loads(ujson.dumps(position.tolist())))
    #velocity_old = np.array(ujson.loads(ujson.dumps(velocity.tolist())))


## main
def main():
    #init_boids()
    global disperse
    global t
    plot_time = 0.0
    while t <= time:
        t = round(t, 2) # round t for ease of use
        plot_time = round(plot_time, 2)
        #print(boids_old==boids)
        #if 100.0 <= t < 150.0:
        #    disperse = True
        move_boids()
        #flocks seem to form around 25 s, intorduce predator
        if t > 25.0:
            move_predator()

        if plot_time >= plot_interval:
            draw_boids(t)
            plot_time = 0.0
        plot_time = plot_time + dt
        t = t+dt

if __name__ == "__main__":
    main()


