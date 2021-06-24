function [xhat_optimal,P_optimal] = KalmanFilter(y_k, phi_last, xhat_last, P_last, Qk, y_last)
%Computes the mean and covariance of x_k|k-1
vk = 1; 
wk = 0.01;
T = 0.1;

Fk = F_jacobian(T, vk, phi_last); 
[xhat_k, P_k] = mean_cov(xhat_last,Fk, P_last, Qk, phi_last, y_last, vk, wk, T);
H_k = H_jacobian(xk,yk); 
[yhat_last,K_k] = measurement_predict(xhat, yk, H_k, P_last, R_k); 
%Estimate: 
xhat_optimal = xhat_k + K_k*(y_k-yhat_last); 
P_optimal = P_k-K_k*H_k*P_k;


end