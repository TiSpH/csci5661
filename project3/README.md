# Project 3: Inverse Kinematics
Group: Zhou Zhuang

[Source code](IK)

# Summary:


# Features:
- Multi-arm IK
  - Left arm and right arm
  - Moving independently

- Joint limits (0:10)
- User Interaction

https://user-images.githubusercontent.com/14122965/141924101-eaeb8440-fb84-4995-9618-371e6840da94.MP4

- Obstacles
  - Using line circle intersection checking algorithm similar to the project 1

https://user-images.githubusercontent.com/14122965/141924256-905fc5fc-c323-4528-a5f8-84a6ffeb0434.MP4

- Alternative IK Solver
  - The first IK solver starts to rotate the end limb first
  - The second IK solver regards the end effector as the root and propagates backwards from the original root
  - Sometimes the second IK solver moves faster than the first one. (0:03)
  - The first one is easier and more flexible to me because it is easir to set limits. While the second one needs interpolation.

https://user-images.githubusercontent.com/14122965/141924236-485c817f-4add-4945-b9bb-8ebb4a1c566e.MP4


# Libraries used
- Processing 3

# Difficulties
- 
- 
