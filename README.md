# Kalman Filter in C

A simple and accurate Kalman filter implementation.  


1. It works stably even for float16 matrices.  
 Because this implementation uses sqrt decomposition.  
  
2. It does NOT depend on the other external library,  
 only using orignal matrix operation library.  

Therefore,  
it is possible to deal with operations even on a weak CPU like embedded system environment.  


## introduce it to environment.

```
git clone --recursive [repository]
```

## Teory
### Normal form Kalman Filter

```math
X ^{t+1}_{4} = 
X _{t+1/t} = F _{t}
```

$$ x_{t+1/t} = F_{t} x_{t}



## Functions and explanations.

### functions
|Function name|Explanations|Arguments|
|:---|:---|:---|
|fKalmanFilter_New|dynamically allocate structure data on memory for computing kalmanfilter process, and initialize thoes values.|int, int : dimention of state and measurement|
|fKalmanFilter_Initialize|initialize values on data.|_stKalmanFilter*|
|fKalmanFilter_Run| Execute measurement update and time update in one time.|_stKalmanFilter*|
|fKalmanFilter_TimeUpdate| Execute time update process.|_stKalmanFilter*|
|fKalmanFilter_PriorEstimate| Predict states on next time step. |_stKalmanFilter*|
|fKalmanFilter_MeasurementUpdate| Execute measurement update process. <br> because it is normal computation on textbook, this process is easy understandable for you.|_stKalmanFilter*|
|fKalmanFilter_MeasurementUpdate_UD| Execute measurement update process using UD-decomposition <br> therefore this process doesn't have much numeric error, you can compute accurately even if it is float16.|_stKalmanFilter*|
|fKalmanFilter_MeasurementUpdate_SQ|Execute measurement update process using Square-decomposition <br> therefore this process doesn't have much numeric error, you can compute accurately. but numeric error is bigger than the process that is using UD-decomposition.|_stKalmanFilter*|

### data structure
this repository is only using "_stKalmanFilter" data structure.
These table on below shows members in "_stKalmanFilter", and its explanations.
You have to define measurement and time update model on this values for Kalman Filter modeling.

|name of members|type|explanation|
|:---|:---|:---|
|vMeasureLengh|int|length of measurement vector|
|vStateLength|int|length of state vector|
|oMeasure|_stMatrix|measurement vector (Mx1)|
|oH|_stMatrix|measurement matrix (SxM)|
|oF|_stMatrix|system update matrix (SxS)|
|oState_pre|_stMatrix|predicted state vector from previous time step (Sx1)|
|oErrCov_pre|_stMatrix|predicted state covaricance matrix from previous time step(SxS)|
|oCovQ|_stMatrix|covariance matrix on system error (SxS)|
|oG|_stMatrix|transformation on system error Q(SxM)|
|oKalmanGain_cor|_stMatrix|Kalman Gain|
|oState_cor|_stMatrix|corrected state vector|
|oErrCov_cor|_stMatrix|corrected covariance matrix on state vector|
|oCovR|_stMatrix|covariance matrix for measurement (MxM)|