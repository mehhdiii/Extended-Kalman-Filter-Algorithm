function [xhat_optimal,P_optimal] = KalmanFilter(y_k, Q_k, R_k, xhat_last, P_last, vk, wk, T)
%Predicts the state xk and covariance matrix pk given k-1 information 

%compute jacobian of system:  
F_k = F_jacobian(T, vk, xhat_last); 

%compute predicted mean and covariance
[xhat_predict, P_k_predict] = state_predict(xhat_last,F_k, P_last, Q_k, vk, wk, T); 

%compute jacobian of sensor: 
H_k = H_jacobian(xhat_predict); 

%predict the measured value
[yhat_predict,K_k] = measurement_predict(xhat_predict, H_k, P_k_predict, R_k); 

%Estimate: 
xhat_optimal = xhat_predict + K_k*(y_k-yhat_predict); 
P_optimal = P_k_predict-K_k*H_k*P_k_predict;


end