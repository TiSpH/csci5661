# Project 1: GeoMetric Methos
Group: Zhou Zhuang

[Source code](Proj1_Test)
# Features:
- Single Agent Navigation
  - A* is used
  - Agents will skip redundant nodes if possible

https://user-images.githubusercontent.com/14122965/135947716-efd39b7f-51aa-4d7a-b09f-78612b97259c.mp4

- Improved Agent & Scene Rendering
  - Agents are rendered as rockets
  - Obstacles are rendered as planets
  - User controled obstacle is rendered as death star
  - Background is space
- Orientation Smoothing 
  - Rockets will move towards where head is pointing to
  - Smooth turn using interpolation (**Did not show in other vidoes. I did this last**)

https://user-images.githubusercontent.com/14122965/135958423-34093929-1bda-428e-8950-61674fd4360b.mp4

- User Scenario Editing
  - Use left/right mouse button to set dest/src
  - Use center mouse button to set death star
  - Use arrow keys to move death star

https://user-images.githubusercontent.com/14122965/135948675-92d73127-61c9-47b5-938a-fb8d0878f4d7.mp4

- Realtime User Interation
  - Agents will avoid to hit death star in real time

https://user-images.githubusercontent.com/14122965/135949704-06700879-570d-43cf-8a19-3ba49c610063.mp4

- Multiple Agents Planning
  - Multiple agents can run simutaniously, the amout can be set by numAgent.

https://user-images.githubusercontent.com/14122965/135950528-e18771fb-bbf3-4309-b583-11aece65c9a9.mp4

- Crowd Simulation
  - Agents will avoid other agents and obstacles
  - TTC is used
  - Interesting senario 1: sometimes (00:16) an agent will get stuck bewteen another agent and obstacles
  - Interesting senario 2: agents will form a circle if they have same destination

https://user-images.githubusercontent.com/14122965/135951496-4176a554-6f1c-448f-b884-c1fe43f0c45d.mp4

https://user-images.githubusercontent.com/14122965/135956290-823c55f8-6910-4c1d-9c8e-6ec052b4bbda.mp4

# Libraries used
- java.util.HashSet
- java.util.PriorityQueue

# Difficulties
- Agents will jitter after reaching the end when TTC is enabled. Simply setting the velocity to zero will affect the application of force.
- Did not find a good way to smooth orientation
