#%%
import numpy as np
import random
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

#position updates: new_pos = old_pos + speed*dt
#speed updates: new_speed = old_speed + accel*dt

#optional: turn rules on and off during runtime

# simulation settings
x_size = 10
y_size = 10
z_size = 10
num_boids = 50
time = 100 #seconds
dt = 1 #seconds

boids = []

# defining our boids
class Boid:
    def __init__(self):
        # each boid has a position vector [x,y,z] and a velocity vector [x,y,z]
        self.position = np.array([random.randint(0,x_size),random.randint(0,y_size),random.randint(0,10)])
        self.velocity = np.array([random.randint(0,10)*0.1,random.randint(0,10)*0.1,random.randint(0,10)*0.1])

# initialize all boids        
def init_boids():
    while len(boids) < num_boids:
        boids.append(Boid())
    return boids

# update visual
def draw_boids():
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.set(xlim=(0,x_size), ylim=(0,y_size), zlim=(0,z_size))
    for b in boids:
        ax.quiver(b.position[0], b.position[1], b.position[2],\
            b.velocity[0],b.velocity[1],b.velocity[2], length=.5, normalize=True)

    plt.show()

# calculate new boid positions based on ruleset
def move_boids():
    for b in boids:
        v1 = cohesion(b)
        v2 = separation(b)
        v3 = alignment(b)
        b.velocity = b.velocity + v1 + v2 + v3
        b.position = b.position + b.velocity*dt
        for i in range(3):
            if b.position[i] >= 10.0:
                 b.position[i] = 0.0



# basic ruleset
# boids try to fly to the centre of mass of neighbouring boids
def cohesion(current_boid):
    vector = np.array([0,0,0])
    for b in boids:
        if b != current_boid:
            vector = vector + b.position
        vector = vector/(len(boids)-1)
    return vector

# boids try not to collide with other boids
def separation(boid):
    vector = np.array([0,0,0])
    
    return vector

# boids try to match velocity/direction with near boids
def alignment(boid):
    vector = np.array([0,0,0])
    
    return vector

# extended ruleset


# main
def main():
    boids = init_boids()
    for b in boids:
        print(b.position)
    
    t=0.0
    while t < time:
        move_boids()
        draw_boids()
        t = t+dt

if __name__ == "__main__":
    main()


# %%
