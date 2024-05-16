# Two barista robot chasing each other.
This is part 2 of the construct's checkpoint 8th: TF2
The barista robot description and gazebo files are in 
- barista_robot_description
- barista_robot_gazebo
repositories. 


Idea to solve part2
1. because the two robots has TF connecting them, thus they know relationship w.r.t each other
2. Write a node that listen to TF of both Rick's and Morty's, and calculate the different
3. The different calculated in 2) will be used to calculate a twist message and pubish onto Rick's cmd_vel
4. 3) is done by timer callback
5. Which type of callback group am I using?
