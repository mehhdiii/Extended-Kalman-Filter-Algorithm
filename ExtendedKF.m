% function [xhat_optimal,P_optimal] = KalmanFilter(y_k, Q_k, R_k, xhat_last, P_last, vk, wk, T)
% %Predicts the state xk and covariance matrix pk given k-1 information 
% 
% %compute jacobian of system:  
% F_k = F_jacobian(T, vk, xhat_last); 
% 
% %compute predicted mean and covariance
% [xhat_predict, P_k_predict] = state_predict(xhat_last,F_k, P_last, Q_k, vk, wk, T); 
% 
% %compute jacobian of sensor: 
% H_k = H_jacobian(xhat_predict); 
% 
% %predict the measured value
% [yhat_predict,K_k] = measurement_predict(xhat_predict, H_k, P_k_predict, R_k); 
% 
% %Estimate: 
% xhat_optimal = xhat_predict + K_k*(y_k-yhat_predict); 
% P_optimal = P_k_predict-K_k*H_k*P_k_predict;
% 
% 
% end


classdef ExtendedKF < handle
   properties
      
      Xhistory
      Plast
      Xtrue
      statetransitionfcn 
      measurementfcn 
      statej
      measurementj
      statecovariance
      measurementcovariance
      T
      xk 
      k
      vk %plant noise
      wk %measurement noise
      state_dim
      measurement_dim
      hasadditivenoise
   end
   
   methods
       
      %constructor of class
      function self = ExtendedKF(statetransition_f, measurement_f,...
              state_j, measurement_j, state_covariance,...
              measurement_covariance, sampling_time, initial_x)
          if nargin == 8
              self.statetransitionfcn = statetransition_f; 
              self.measurementfcn = measurement_f; 
              self.statej = state_j; 
              self.measurementj = measurement_j; 
              self.statecovariance = state_covariance; 
              self.measurementcovariance = measurement_covariance; 
              self.T = sampling_time; 
              
              self.state_dim = size(intial_x); 
              self.Xtrue(:, 1) = initial_x; 
              self.Xhistory = zeros(state_dim); 
              self.k = 1; 
              self.hasadditivenoise = true; 
              self.Xtrue = zeros(state_dim); 
          end
          
      end
      
      
      function [Xpred, Ppred] = predict(self) 
         %create plant noise
         self.vk = sqrt(self.statecovariance)*randn(self.state_dim, 1); 
         %create noisy plant state: REFERENCE VALUE
         xtrue_last = self.Xtrue(:, end); 
         self.xk = self.statetransitionfcn(xtrue_last, self.T) + self.vk;
         
         xhat_last = self.Xhistory(:, end); 
         F = self.statej(self.T, self.vk, xhat_last); 
         Xpred = self.statetransitionfcn(xhat_last, self.T); 
         Ppred = F*self.Plast*F' + self.statecovariance; 
         
         %save the new values: 
         self.k = self.k+1; 
         self.Plast = Ppred;
         self.Xhistory(:, self.k) = Xpred;  
         self.Xtrue(:, self.k) = self.xk; 
         

      end
      
      function [Xcorr, Pcorr] = correct(self) 
         %create measurement noise
         self.wk = sqrt(self.measurementcovariance)*randn(self.state_dim, 1); 
         %create true measurement
         yk = self.measurementfcn(self.xk) + self.wk; 
         
         %correcting measurement
         Xpred = self.Xhistory(:, self.k); 
         H = self.measurementj(Xpred); 
         Ypred = self.measurementfcn(Xpred); 
         Sk = H*self.Plast*H' + self.measurementcovariance; 
         Kk = self.Plast*H'*inv(Sk); 
         
         
         
         %correct the readings 
         Xcorr = Xpred+Kk*(yk-Ypred);
         Pcorr = self.Plast - Kk*H*self.Plast;
         
         
         %overwrite to existing values: 
         self.Plast = Pcorr; 
         self.Xhistory(:, self.k) = Xcorr;  

      end
      
   end
end