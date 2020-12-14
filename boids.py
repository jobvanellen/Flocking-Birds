import numpy as np
import random

# simulation settings
x_size = 1000
y_size = 1000
z_size = 0
num_boids = 50
time = 1000 #seconds
dt = 0.001 #1 millisecond

boids = []

# defining our boids
class Boid:
    def __init__(self):
        self.position = np.array([random.randint(0,x_size),random.randint(0,y_size),random.randint(0,z_size)], dtype= tuple)
        
def init_boids():
    boids = []
    while len(boids) < num_boids:
        boids.append(Boid())
    return boids


def draw_boids():
    pass

def move_boids():
    for b in boids:
        cohesion(b)
        separation(b)
        alignment(b)

# basic ruleset
def cohesion(boid):
    pass

def separation(boid):
    pass

def alignment(boid):
    pass


def main():
    boids = init_boids()
    for b in boids:
        print(b.position)
    t=0.0

    while t < time:
        move_boids()
        
        t = t+dt

if __name__ == "__main__":
    main()

