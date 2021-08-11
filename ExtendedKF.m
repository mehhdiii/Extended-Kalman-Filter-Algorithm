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
              
              self.state_dim = size(self.statecovariance(:, 1)); 
              self.measurement_dim = size(self.measurementcovariance(:, 1)); 
              self.Xtrue(:, 1) = initial_x; 
              self.Xhistory = zeros(self.state_dim); 
              self.k = 1; 
              self.hasadditivenoise = true; 
              self.Plast = eye(self.state_dim(1)); 
          end
          
      end
      
      
      function [Xpred, Ppred] = predict(self) 
         %create plant noise
         self.vk = sqrt(self.statecovariance)*randn(self.state_dim(1), 1); 
         %create noisy plant state: REFERENCE VALUE
         xtrue_last = self.Xtrue(:, end); 
         self.xk = self.statetransitionfcn(xtrue_last, self.T) + self.vk;
         
         xhat_last = self.Xhistory(:, end); 
         F = self.statej(xhat_last, self.T); 
         Xpred = self.statetransitionfcn(xhat_last, self.T); 
         Ppred = F*(self.Plast)*F' + self.statecovariance; 
         
         %save the new values: 
         self.k = self.k+1; 
         self.Plast = Ppred; 
         self.Xhistory(:, self.k) = Xpred;  
         self.Xtrue(:, self.k) = self.xk; 
         

      end
      
      function [Xcorr, Pcorr] = correct(self) 
         %create measurement noise
         self.wk = sqrt(self.measurementcovariance)*randn(self.measurement_dim(1), 1); 
         %create true measurement
         yk = self.measurementfcn(self.xk, self.T) + self.wk; 
         
         %correcting measurement
         Xpred = self.Xhistory(:, self.k); 
         H = self.measurementj(Xpred, self.T); 
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