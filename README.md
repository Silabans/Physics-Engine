# Physics Simulation
A program that simulates rigid body interactions due to mechanical physics

## Why am I making a custom physics engine from scratch?
This project is at the intersection of my two greatest interests: Physics and Programming. Building this project teaches me about clean code architecture as it forces me to create readable code and find a way to modularise it, considering the sheer size of the codebase. Moreover, revolving the physics part of the project, it helps me implement textbook concepts into a real, visual project that reinforces my understanding of mechanical physics and teach me more about it (especially regarding rotational motion and angular impulse and inertia).

## What the program aims to handle:
* Collisions (contact forces), by calculating the impulse along the normal of the collision
* Non-contact forces, such as gravitational force and air resistance
* Smooth and consistent frame rendering and physics updating

## Problems that are in Work-in-Progress
* ~~SAT box collisions (impulse resolution)~~
* ~~Rotational box physics~~
* Packaging the engine into a modular format
* Using the engine to build a simple "Angry Birds" clone

### Matrix Angle Transformation (Unrotation)
![alt text](<Screenshot 2026-08-23 153058.png>)


### Why I am creating a clone of the "Angry Birds" game
I see this as the final challenge for this project, serving as a test for my custom physics engine to see whether it is developed and polished enough such that it could be used as a foundation for a physics-based game.

This will also serve as a learning point to more clearly uncover the limitations of the physics engine, so that I can focus on the pressing issues of the engine later on to improve its stability and accuracy.