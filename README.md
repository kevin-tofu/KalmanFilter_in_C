# Kalman Filter in C

A simple and accurate Kalman filter implementation.  


1. It works stably even for float16 matrices.  
 Because this implementation uses sqrt decomposition.  
  
2. It does NOT depend on the other external library,  
 only using orignal matrix operation library.  

Therefore,  
it is possible to deal with operations even on a weak CPU like embedded system environment.  


# Teory
## Normal form Kalman Filter

```math
X ^{t+1}_{4} = 
X _{t+1/t} = F _{t}
```

$$ x_{t+1/t} = F_{t} x_{t}



# Functions and explanations.

|Function name|Explanations|
|:---:|:---|
|fKalmanFilter_New|dynamically allocate structure data on memory for computing kalmanfilter process, and initialize values on data.|
|fKalmanFilter_Initialize|initialize values on data.|
|fKalmanFilter_Run| Execute measurement update and time update in one time.|
|fKalmanFilter_TimeUpdate| Execute time update process.|
|fKalmanFilter_PriorEstimate| path to data. |
|fKalmanFilter_MeasurementUpdate| Execute measurement update process. <br> because it is normal computation on textbook, this process is easy understandable for you.|
|fKalmanFilter_MeasurementUpdate_UD| Execute measurement update process using UD-decomposition <br> therefore this process doesn't have much numeric error, you can compute accurately even if it is float16.|
|fKalmanFilter_MeasurementUpdate_SQ|Execute measurement update process using Square-decomposition <br> therefore this process doesn't have much numeric error, you can compute accurately. but numeric error is bigger than the process that is using UD-decomposition.|

