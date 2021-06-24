

%call filter in loop 
[xhat_optimal,P_optimal] = KalmanFilter(y_k, phi_last, xhat_last, P_last, Qk, y_last)