# DirectionAnalysisBot
![image](https://github.com/ChoiSeongHo-h/DirectionAnalysisBot/assets/72921481/12f3f7d3-a45f-432b-a115-7f2f04be2e69)
## Overview
Robots use **different promotional strategies** depending on the **direction pedestrians are walking in**
- Recording pedestrian trace in top view
- Determining pedestrian walking direction 

## 1 Recording pedestrian trace in top view
![image](https://github.com/ChoiSeongHo-h/DirectionAnalysisBot/assets/72921481/7e4cec5d-264b-41c9-a7ed-64ad37ed86ab)

**Recording footprints** in the top view **before determining** the pedestrian's **walking direction**.

The RGB image is converted to bounding boxes and object indices through a deep sort. The converted information is combined with the depth image and recorded as traces in the top view.

### 1.1 Determining bounding boxes and object indices 
The RGB image is passed through a deep sort object tracking model. This extracts pedestrians and their unique IDs from the image. 

### 1.2 Determining the depth of the bounding box
![Untitled](https://user-images.githubusercontent.com/72921481/146730922-8b5d0a05-e7e7-4c50-a948-526c65690e01.png)

- Red box: Bounding box projected onto depth image
- Black line: Pixel(depth) extraction line
  
I want to determine the depth of the bounding box extracted from a deep sort. I use the following method:
#### 1.2.1 Step 1: Extracting representative values per line 
1. copy the pixel extraction line as a vector.
2. sort the vectors in ascending order.
3. select the pixel value located 35% to the left as a representative value for the line.

Pedestrians are perceived by the depth camera to be closer than the background, so they have a small depth value, which is also represented by a small value in the extracted line. With this logic, I set the value 35% to the left of the sorted lines (small depth value, close distance) as the representative value of one line.

If the pedestrian occupies less than 35% of each line, the depth of the background is extracted. 

This is similar to the OS-CFAR method of radar signal detection. 

#### 1.2.2 Step 2: Determining the representative depth of a bounding box based on representative values per line
1. take the lowest representative value extracted from each line.
2. If the extracted value is an outlier (depth == 0.0), take another value.

The background has a larger value than the pedestrian, so the minimum value is chosen to remove the depth of the background.

#### 1.2.3 Understanding the algorithm
The algorithm is driven by the assumption that any of the lines are occupied by pedestrians more than 35% of the time. This assumption is reasonable because the width of the bounding box reflects the width of the pedestrian. 

There is no line at the top of the bounding box because it is unlikely that a pedestrian's head would occupy more than 35% of the line. There is also no line at the bottom of the bounding box. This is because the occupancy of the pedestrian's feet is similarly small, and the small depth difference between the background and the pedestrian is confusing.  There are margins on either side of each line segment, as interference between multiple bounding boxes can occur.

The above algorithm improves speed by using rowwise operations and dramatically reduces computation by using sparse checks. It is about 10 times faster than clustering after analysing the histogram of bounding boxes.

### 1.3 Getting traces based on the depth of a bounding box
The robot records the position of the pedestrian in the top view based on the position of the bounding box in the image, the camera's angle of view, and the depth of the bounding box. Traces for each pedestrian are recorded based on the pedestrian's index.

Lens distortion can be taken into account to more accurately record the pedestrian's position.


## 2 Determining pedestrian walking direction
![image](https://github.com/ChoiSeongHo-h/DirectionAnalysisBot/assets/72921481/b69e429f-91d1-41c5-ba14-5d7c2fb5855a)
- Green line: Entrance to a shop
- Blue line: Robot's field of view
- Left: A pedestrian stops near the robot and shows interest in it.
- Right : The pedestrian is leaving the store

**Determining** the **state** and **direction** of a **pedestrian** based on traces.

### 2.1 Determining whether to stop a pedestrian using trace covariance  
The robot calculates the covariance of the pedestrian's trace. If the determinant of the covariance is small but near the robot, the robot determines that the pedestrian is interested in the robot. If the covariance matrix is large, the robot believes that the pedestrian is moving.

### 2.2 Determine a pedestrian's principal walking axis  
If the determinant of the trace covariance is large, the robot believes that the pedestrian is moving.
The robot uses the following procedure to determine the main walking axis:
1. the robot applies PCA to the traces of the moving pedestrian and compares the magnitudes of the principal and minor components.

2. If the principal component is significantly larger than the minor component, the robot determines that the person is moving linearly and is aiming in a "certain" direction. 

3. Finally, the direction of the principal component is determined as the principal walking axis. The principal walking axis is represented by the light blue straight line in the figure on the right above. 

### 2.3 Determining whether pedestrians enter or leave a store 
The robot determines that the pedestrian is entering or leaving the store when the pedestrian's main walking axis intersects the store entrance.
The store entrance is represented by the green line in the figure above. 
However, the above method cannot distinguish whether the pedestrian is entering the store or leaving the store, so the robot determines this by dot producting the following two items:
1. the vector from the centre of the trace to the centre of the store entrance
2. the normalised sum of all vectors from the adjacent time t-1 trace element to the time t trace element.

If the value of the dot product is close to 1 (the direction is the same), it means that the pedestrian is entering the store, and if it is close to -1 (the direction is opposite), it means that the pedestrian is exiting the store.

If the robot tries to determine the direction using only the earliest and latest elements of the trace, as opposed to the above method, the direction determination will be very unstable. It has been experimentally shown that the above vector normalisation method increases the robustness of the direction determination.

## 3 Configuration of the robot 

![KakaoTalk_20220614_005502877](https://user-images.githubusercontent.com/72921481/173395129-4c056026-3d36-449c-82f0-d642726eeaef.png)

