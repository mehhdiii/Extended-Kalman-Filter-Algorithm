function [yhat_last,K_k] = measurement_predict(xhat_k, yk, H_k, P_last, R_k)

yhat_last = sqrt(xhat_k^2 + yk^2); 
S_k = H_k*P_last*H_k' + R_k; 
K_k = P_last*H_k'*inv(S_k); 

end

