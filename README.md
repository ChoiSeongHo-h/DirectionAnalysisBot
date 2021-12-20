# Human_Direction_Tracking_Robot_ROS

![KakaoTalk_20211203_174619626](https://user-images.githubusercontent.com/72921481/146729196-e181d587-d2dc-45db-8ab8-7606a2708b46.jpg)
![KakaoTalk_20211204_012259209_01](https://user-images.githubusercontent.com/72921481/146729243-312b4493-6f36-4899-b0fc-91f4405270e0.jpg)


As shown in the image above, it detects the status of people near the robot.


It tracks people who are near the robot, and who is heading for or exiting a certain section.


In this system, the result of object tracking such as deepsort and 16-bit depth image must be input.


This robot estimates the distance to the person based on the object tracking result and the depth image.


It shows a human trace in a 2d top view.


The robot can also track people's directions to know if they're going into or out of a specific area.


First, get a 16-bit depth image. The pixel value is the distance in mm.


For speed, this data is obtained by reference.


![Untitled](https://user-images.githubusercontent.com/72921481/146730922-8b5d0a05-e7e7-4c50-a948-526c65690e01.png)


Then, a representative value of the distance of the person is obtained using the detection result and the depth value.


There will be a method of averaging the bounding area, etc.


But humans don't even fill the bounding box, and the average is vulnerable to outliers.


Therefore, a value similar to the median value will be used as a representative of human distance.


Do not use median values. We will use the value that exists in the top 35% idx.


In the figure above, the process is as follows.


1. for (line in lines)

  1. Copies the line's data into a vector. → We can refer to it for computational benefit, but we will reuse the depth later, so we copy it.

  2. Sort the vectors in ascending order.

  3. Take the 35% point idx as a representative value of one line. Even if an outlier (0) exists, the outlier will not be taken unless it accounts for more than 35%.

  Humans will be perceived as closer than the background by the camera, so there is a high probability that they are within 35% of the alignment.
  
  Therefore, the 35% point value will be extracted from the human distance value. 
  
  If the human distance component in the black line in the figure is less than 35%, the distance value of the background will be taken.
  
2. The minimum non-zero value among the representative values ​​of each line is taken as the final representative value of the person.

  Even if there are several line segments with a human component of less than 35% (there will be a distant background at the 35% point, so the representative value of the line segment is large). Since the minimum of the line segment representatives is taken, the probability of taking the human representative is high.
  
  The width of the bounding box is determined by the width of the person. The tighter the spacing between the lines, the higher the probability of taking the human representative value.
  
The reason why there is no line at the top → The tip of the head is less likely to be occupied by a person more than 35%
  
The reason why there is no line segment below → Because the depth of the floor closer than the person can be taken
  
The reason why there is no line segment to both sides → Because interference between multiple bounding boxes may occur
  
The above algorithm aims to improve the speed by row-direction operation, and reduces the amount of computation dramatically by checking sparsely.
  
Disadvantages of this algorithm: If there is an obstacle in front of a human, the depth of the obstacle is received.



Next, we know the pixel position and distance value of detection, so if we only know the camera angle of view, we can know the angle of the camera and the object.

Also, if you know the angle and distance, you can express the position of the object in the top view.

In the same way as above, traces of people can be found and statistical calculations can be performed.

Find the correlation coefficient to find the linearity of the trace. If the correlation coefficient is high, the person is walking towards somewhere.

If it is difficult to obtain a correlation coefficient by walking in the horizontal or vertical direction, it is judged that the person is walking horizontally/vertically if it is sxx>>syy or syy>>sxx.

Based on the previously obtained statistical data, linear regression is performed and it is determined whether the regression line passes through a specific area.

Find the intersections point and average of traces.

The direction of the mean at the intersection is the outgoing direction.

The trace of a person is stored as a vector, and the more past data it is, the more it is stored on the left.

Normalize the vector between the k+1th trace from the kth trace, add them all up, and normalize again. If so, this vector will be a vector representing the direction from the past to the present.

Dot product with the vector from the intersection to the mean above.

If it is close to 1, it is out, and if it is close to -1, it is in.

There may be other methods other than the above normalization method for calculating the progress direction.

For example, to find the direction of the sage's trace from the most past trace.

However, these methods are vulnerable to outliers.

The method I used for averaging the normalized vectors is very robust against outliers. This is because only simple directions are indicated.

The trace of a person is stored in a vector, and this vector is stored in a vector again by index.

The position of the person is determined relative to the robot.

Since robots exist in an absolute coordinate system (arbitrary latitude, longitude, and direction), humans are also fixed on the absolute coordinate system.

Naturally, therefore, even if the robot moves or rotates, the human trace does not change.

I made a robot that promotes the facilities inside the library through the above program.

By monitoring the entry and exit, different methods of publicity were applied to those who went out and those who came in.
