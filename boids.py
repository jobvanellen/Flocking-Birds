import numpy as np
import random
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

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
x_size = 200
y_size = 200
z_size = 200

num_boids = 50
time = 1000.0 # seconds, total simulation time
dt = .05 # seconds, simulation timestep
plot_interval = 1.0 # seconds

boids = []
boids_old = []
goal = np.ones(3)*x_size/2
print(goal)

cohesion_area = 30
alignment_area = 15
separation_area = 5

disperse = False # allow for scattering behaviour if true

# defining our boids
class Boid:
    def __init__(self):
        # each boid has a position vector [x,y,z] and a velocity vector [x,y,z]
        self.position = np.array([random.randint(0,x_size),random.randint(0,y_size),random.randint(0, z_size)])
        self.velocity = np.array([random.randint(-10,10),random.randint(-10,10),random.randint(-10,10)])

# setting up plot stuff
plt.ion()
fig = plt.figure()
ax = fig.gca(projection='3d')

# initialize all boids        
def init_boids():
    while len(boids) < num_boids:
        b = Boid()
        boids.append(b)
    copy_boids()


## core functions
# update visual
def draw_boids(t):
    ax.clear()
    ax.set(xlim=(-10,x_size+10), ylim=(-10,y_size+10), zlim=(-10,z_size+10))
    ax.scatter(goal[0], goal[1], goal[2], color="green")
    for b in boids:
        ax.quiver(b.position[0], b.position[1], b.position[2],\
            b.velocity[0],b.velocity[1],b.velocity[2], length=4, normalize=True)
    timestring = str(t)
    ax.text(0,0,0, timestring)
    plt.show(block = False)
    plt.pause(0.0001)


# calculate new boid positions based on ruleset
def move_boids():
    #vector multipliers
    m1 = 0.5 # cohesion
    m2 = 1.0 # separation
    m3 = 2.0 # alignment
    m4 = 1.0 # bound position
    m5 = 1.0 # flock to goal

    if disperse: # if disperse flip m1
        m1 = m1*-1

    for b in boids:
        v1 = m1 * cohesion(b)
        v2 = m2 * separation(b)
        v3 = m3 * alignment(b)
        v4 = m4 * bound_position(b)
        v5 = m5 * flock_to_goal(b)


        b.velocity = b.velocity + (v1 + v2 + v3 + v4 + v5)*dt
        b.velocity = limit_velocity(b)
        b.position = b.position + b.velocity*dt

    copy_boids()

## basic ruleset
# boids try to fly to the centre of mass of neighbouring boids
# metre/second/pos_diff
def cohesion(current_boid):
    v = np.zeros(3)
    num = 0
    for b in boids_old:
        if in_range(current_boid, b, cohesion_area):
            v = v + b.position
            num = num + 1
        if num > 0:
            v = v/num
    return (v - current_boid.position) / 100

# boids try not to collide with other boids
def separation(current_boid):
    v = np.zeros(3)   
    for b in boids_old:
        if in_range(current_boid, b, separation_area):
            v = v - (b.position - current_boid.position)
    return v

# boids try to match velocity with near boids
def alignment(current_boid):
    v = np.zeros(3)
    num = 0    
    for b in boids_old:
        if in_range(current_boid,b, alignment_area):
            v = np.add(v,b.velocity)
            num = num + 1
        if num > 0:
            v = v/num
    return (v - current_boid.velocity)/8

## extended ruleset
# change course to stay inside world
def bound_position(current_boid):
    v = np.zeros(3)
    factor = 10 # steering influence level

    if current_boid.position[0] < 0.0:
        v[0] = factor
    elif current_boid.position[0] > x_size:
        v[0] = -1 * factor

    if current_boid.position[1] < 0.0:
        v[1] = factor
    elif current_boid.position[1] > y_size:
        v[1] = -1 * factor

    if current_boid.position[2] < 0.0:
        v[2] = factor
    elif current_boid.position[2] > z_size:
        v[2] = -1 * factor

    return v

def flock_to_goal(current_boid):
    return (goal - current_boid.position) / 100


## helper functions
# checks if boid is within range r of other boid
def in_range(b1, b2, r):
    #if b1 == b2:
        #print("self found")
        #return False
    if abs(b1.position[0]-b2.position[0]) > r:
        return False
    if abs(b1.position[1]-b2.position[1]) > r:
        return False
    if abs(b1.position[2]-b2.position[2]) > r:
        return False
    return True

def limit_velocity(b):
    vlim = 10
    v = b.velocity
    
    for i in range(0,3):
        if abs(b.velocity[i]) > vlim:
            v[i] = (b.velocity[i] / abs(b.velocity[i])) * vlim 
    return v

def copy_boids():
    global boids_old
    boids_old = []
    for b in boids:
        boids_old.append(b)
    #print(boids_old[0])


## main
def main():
    init_boids()
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
