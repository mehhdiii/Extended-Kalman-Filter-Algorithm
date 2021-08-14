classdef ExtendedKF < handle
   properties
      
      predhistory
      truehistory
      Plast
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
      measurementhistory
   end
   
   methods
       
      %constructor of class
      function self = ExtendedKF(statetransition_f, measurement_f,...
              state_j, measurement_j, state_covariance,...
              measurement_covariance, sampling_time, initial_x, additivenoise)
          if nargin == 9
              self.statetransitionfcn = statetransition_f; 
              self.measurementfcn = measurement_f; 
              self.statej = state_j; 
              self.measurementj = measurement_j; 
              self.statecovariance = state_covariance; 
              self.measurementcovariance = measurement_covariance; 
              self.T = sampling_time; 
              
              self.state_dim = size(self.statecovariance(:, 1)); 
              self.measurement_dim = size(self.measurementcovariance(:, 1)); 
              self.truehistory(:, 1) = initial_x; 
              self.predhistory = zeros(self.state_dim); 
              self.measurementhistory = zeros(self.measurement_dim); 
              self.k = 1; 
              self.hasadditivenoise = additivenoise; 
              self.Plast = eye(self.state_dim(1)); 
          end
          
      end
      
      
      function [Xpred, Ppred] = predict(self) 
         %create plant noise
         self.vk = sqrt(self.statecovariance)*randn(self.state_dim(1), 1); 
         %create noisy plant state: REFERENCE VALUE
         xtrue_last = self.predhistory(:, end); 
         
         xtrue = self.statetransitionfcn(xtrue_last, self.T, 0); 
         self.xk =  self.statetransitionfcn(xtrue_last, self.T, self.vk);
         
         xhat_last = self.predhistory(:, end); 
         F = self.statej(xhat_last, self.T); 
         Xpred = self.statetransitionfcn(xhat_last, self.T, 0); 
         Ppred = F*(self.Plast)*F' + self.statecovariance; 
         
         %save the new values: 
         self.k = self.k+1; 
         self.Plast = Ppred; 
         self.predhistory(:, self.k) = Xpred;  
         self.truehistory(:, self.k) = xtrue; 
         

      end
      
      function [Xcorr, Pcorr] = correct(self) 
         %create measurement noise
         self.wk = sqrt(self.measurementcovariance)*randn(self.measurement_dim(1), 1); 
         %create true measurement
         yk = self.measurementfcn(self.xk, self.T, self.wk); 
         
         %correcting measurement
         Xpred = self.predhistory(:, self.k); 
         H = self.measurementj(Xpred, self.T); 
         Ypred = self.measurementfcn(Xpred, self.T, 0); 
         Sk = H*self.Plast*H' + self.measurementcovariance; 
         Kk = self.Plast*H'*inv(Sk); 
         
         
         
         %correct the readings 
         Xcorr = Xpred+Kk*(yk-Ypred);
         Pcorr = self.Plast - Kk*H*self.Plast;
         
         
         %overwrite to existing values: 
%          self.measurementhistory(:, self.k) = yk; 
         self.Plast = Pcorr; 
         self.predhistory(:, self.k) = Xcorr;  

      end
      
   end
end