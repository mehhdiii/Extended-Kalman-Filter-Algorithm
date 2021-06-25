function [xhat_optimal,P_optimal] = KalmanFilter(y_k, Q_k, R_k, xhat_last, P_last, vk, wk, T)
%Predicts the state xk and covariance matrix pk given k-1 information 

%compute jacobian of system:  
F_k = F_jacobian(T, vk, xhat_last); 

%compute predicted mean and covariance
[xhat_k_last, P_k_last] = mean_cov(xhat_last,F_k, P_last, Q_k, vk, wk, T); 

%compute jacobian of sensor: 
H_k = H_jacobian(xhat_k_last); 

%predict the measured value
[yhat_k_last,K_k] = measurement_predict(xhat_k_last, H_k, P_k_last, R_k); 

%Estimate: 
xhat_optimal = xhat_k_last + K_k*(y_k-yhat_k_last); 
P_optimal = P_k_last-K_k*H_k*P_k_last;


end