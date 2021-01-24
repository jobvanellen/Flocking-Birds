# Bird flocking simulation using Boids
## Engineering fundamentals assessment project

### Main Ruleset
The Boids algorithm is comprised of three central behavioural traits:

1. Separation: Steering to avoid crowding local flockmates
2. Alignment: Steering towards the average heading of local flockmates
3. Cohesion: Steering to move towards the average position (center of mass) of local flockmates
 
### Extended Ruleset
1. Boids stay within world boundaries
2. Boids avoid predators

### Predator rules
1. Stays within world boundaries
2. Finds and chases nearest boid
