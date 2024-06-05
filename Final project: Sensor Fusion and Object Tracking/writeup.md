# Writeup: Track 3D-Objects Over Time

Please use this starter template to answer the following questions:

### 1. Write a short recap of the four tracking steps and what you implemented there (filter, track management, association, camera fusion). Which results did you achieve? Which part of the project was most difficult for you to complete, and why?

#### Tracking steps 
##### Filter
- Implemented Predict step using F and Q function with 6D state X and P as it is a 3D object tracking using constant velocity model.
- Implemented Update step as well as the gamma and S by calling the H and hx.
  
##### Track Management
- Done initialization, updation and deletion of tracks.
- Initialized tracks using measurement input, converted from vehicle to sensor coordinates and state is initialized.
- For managing the tracks, window is used for decreasing the value for unassigned tracks. Deleted tracks if the score is too low compared to delete threshold or P is too big.
- In handling updated tracks, score is increased and state set to tentative or confirmed based on threshold.
- Deleted initialized or tentative tracks only if track.P[0,0] or track.P[1,1] is bigger than params.max_P, or if the score gets very low.
  
##### Association
- MHB distance, gating using chi-square method are implemented and used for implemeting single nearest neighbour data association.
- Find and delete the entry corresponding to the minimum value. Remove that track from unassigned tracks and unassigned measurement.
- If no association is found, return Nan, else return association pair.
  
##### Camera Fusion
- Implement in_fov() function to check whether the input state vector X is seen by the sensor - lidar and camera
- Implement the function get_hx() with the nonlinear camera measurement function h as follows:
a. transform position estimate from vehicle to camera coordinates,
b. project from camera to image coordinates,
c. make sure to not divide by zero, raise an error if needed and then return h(x).
- Given access for camera and initialized camera measurements like z, R and sensor object.
  
#### Results

RMSE plots and Tracking video

RMSE STEP 1: ![RMSE plot](https://github.com/Dharun235/Udacity-solutions-self-driving-car-nanodegree/blob/main/Final%20project%3A%20Sensor%20Fusion%20and%20Object%20Tracking/rmse_step1.png)

RMSE STEP 2: ![RMSE plot](https://github.com/Dharun235/Udacity-solutions-self-driving-car-nanodegree/blob/main/Final%20project%3A%20Sensor%20Fusion%20and%20Object%20Tracking/rmse_step2.png)

RMSE STEP 3: ![RMSE plot](https://github.com/Dharun235/Udacity-solutions-self-driving-car-nanodegree/blob/main/Final%20project%3A%20Sensor%20Fusion%20and%20Object%20Tracking/rmse_step3.png)

RMSE STEP 4: ![RMSE plot](https://github.com/Dharun235/Udacity-solutions-self-driving-car-nanodegree/blob/main/Final%20project%3A%20Sensor%20Fusion%20and%20Object%20Tracking/rmse_step4.png)

Tracking video step 4 is available in this google drive link - [Tracking video](https://drive.google.com/file/d/1KP0B5zo8-r7LBNPX6fhQCsa5NSQD7oe4/view?usp=sharing)

Inference:
From the results it is evident that mean RMSE is lesser than 0.25 for all the steps and the rmse plot for step 3 and the tracking video shows no confirmed ghost tracks or track losses occured. Two of the tracks are tracked from beginning to end of the sequence (0s - 200s) without track loss. 

Completing the track management was difficult because I couldn't figure out how much to reduce the track score, otherwise it was easier to do the project.

### 2. Do you see any benefits in camera-lidar fusion compared to lidar-only tracking (in theory and in your concrete results)? 
Sensor fusion always results in increased prediction. Using the combined non-linear velocity model from camera measurements and linear motion model from Lidar both in theory and in results resulted in lesser RMSE and better track management.

### 3. Which challenges will a sensor fusion system face in real-life scenarios? Did you see any of these challenges in the project?
- Sensor calibration
- Noise and fault tolerance issues
- Environmental variability 
- Quick decision making in real time is lower
- Occlusions and sensor limitations
- Data associations

### 4. Can you think of ways to improve your tracking results in the future?
- Preprocessing and filtering techniques can be used
- Feature extraction can be intensly used
- better data association techniques can be used
- Incorporating multiple sensors and fusing their values can result in much more higher accuracy
- Creating a better and accurate models of environment like bicycle model for cars
