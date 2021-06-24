function [xhat_k,P_k] = mean_cov(xhat_last,Fk, P_last, Qk, phi_last, y_last, vk, wk, T)
%Computes the mean and covariance of x_k|k-1

xhat_k = [xhat_last+T*vk*cos(phi_last); y_last+T*vk*sin(phi_last); phi_last+T*wk];
P_k = Fk*P_last*Fk'+Qk;
end

