#%%
import numpy as np
import random
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
#from matplotlib import animation

# position updates: new_pos = old_pos + speed*dt
# speed updates: new_speed = old_speed + accel*dt

# optional: turn rules on and off at runtime

# simulation settings
x_size = 100
y_size = 100
z_size = 100
num_boids = 50
time = 1000 # seconds
dt = .5 # seconds

boids = []

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
        boids.append(Boid())
    return boids

# update visual
def draw_boids():
    ax.clear()
    ax.set(xlim=(-10,x_size+10), ylim=(-10,y_size+10), zlim=(-10,z_size+10))
    for b in boids:
        ax.quiver(b.position[0], b.position[1], b.position[2],\
            b.velocity[0],b.velocity[1],b.velocity[2], length=4, normalize=True)
    plt.show(block = False)
    plt.pause(0.0001)

    

# calculate new boid positions based on ruleset
def move_boids():
    for b in boids:
        v1 = cohesion(b)
        v2 = separation(b)
        v3 = alignment(b)
        v4 = bound_position(b)
        b.velocity = b.velocity + (v1 + v2 + v3 + v4)*dt
        b.position = b.position + b.velocity*dt

        #limits = [x_size, y_size, z_size]
        #for i in range(3):
        #    b.position[i] = b.position[i] % limits[i]

# basic ruleset #
# boids try to fly to the centre of mass of neighbouring boids
def cohesion(current_boid):
    v = np.zeros(3)
    for b in boids:
        if b != current_boid:
            v = v + b.position
        v = v/(num_boids-1)
    return v

# boids try not to collide with other boids
def separation(current_boid):
    v = np.zeros(3)
    for b in boids:
        if b != current_boid:
            for i in range(3):
                if abs(b.position[i] - current_boid.position[i]) < 1:
                    v = v - (b.position[i] - current_boid.position[i])
    return v

# boids try to match velocity/direction with near boids
def alignment(current_boid):
    v = np.zeros(3)
    for b in boids:
        if b != current_boid:
            v = v + b.velocity
        v = v / (num_boids-1)
    return (v - current_boid.velocity)/10

# extended ruleset
# change course to stay inside world
def bound_position(current_boid):
    v = np.zeros(3)
    factor = 3

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


# main
def main():
    boids = init_boids()
    for b in boids:
        print(b.velocity)
    
    t=0.0
    while t < time:
        move_boids()
        draw_boids()
        t = t+dt

if __name__ == "__main__":
    main()


# %%
