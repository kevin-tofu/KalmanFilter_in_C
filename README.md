# Kalman Filter in C

A simple and accurate Kalman filter implementation.  

## Features
1. It works stably even for float16 matrices.  
 Because this implementation uses UD-decomposition.  
  
2. It does NOT depend on the other external library,  
 only using orignal matrix operation library.  

Therefore,  
it is possible to deal with operations even on a weak CPU like embedded system environment.  


## Introduce to environment.
```
git clone --recursive [repository]
```

## Theory
### Normal form Kalman Filter

The Kalman filter is formulated as follows.  
The 1st formula is called a system equation that describes how system variable is going to transition, 
The 2nd formula is the so-called measurement equation. it describes how we can observe the state of information.  

s state random variable that is hidden information we want to estimate, 
m is measure random variable that we are able to get from the sensor, 
w is the Gaussian white noise of which characteristic varies on each time step.
  
<img src="https://github.com/kohei-tofu/KalmanFilter_in_C/blob/master/img/eq1.jpg" alt="eq1" title="formulation1">
  
If it takes an expectation between w, v and itself, each co-variance matrix shows Q and R.
and no-correlation between them.  
  
<img src="https://github.com/kohei-tofu/KalmanFilter_in_C/blob/master/img/eq2.jpg" alt="eq2" title="formulation2">
  
 We can formulate Kalman filter based on the above formulation.  
The 1st equation is executed on time-update. The program predicts state variables on the next time step based on the previous one.  
The 1st equation is executed on measurement-update. The program predicts state variables now. It corrects predicted variables based on measured information. 


<img src="https://github.com/kohei-tofu/KalmanFilter_in_C/blob/master/img/filter_eq1.jpg" alt="filter_eq1" title="filter_eq1">

 These matrices are Kalman Gain and Covariance matrix that is updated on time-update and measurement-update.  
It shows how much uncertain information the values are. You are able to run program if you set the covariance matrix properly based on probability.  
  
<img src="https://github.com/kohei-tofu/KalmanFilter_in_C/blob/master/img/filter_eq2.jpg" alt="filter_eq2" title="filter_eq2">

### Square root-Kalman Filter
 Normal Kalman filter formulation caused problems in terms of numeric stability.  
So, Potter et al suggested a square root filter ("Discrete square root filtering: A survey of current techniques") that improves the stability of Kalman filter computation.

### UD decomposition-Kalman Filter
 After Square root techniques were suggested, many kinds of research have arisen.
One solution is "UD decomposition Filter".  
 Compared with a normal Kalman filter, this method is numerically stable even with float 16 types. 
 It is because the normal Kalman filter highly affected by the numeric round error.
 Also, flops are the same as a normal Kalman filter.  Basically, UD decomposition method has advantage.

## Functions and explanations.

### Functions
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

### Data structure
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