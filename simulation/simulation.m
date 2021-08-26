close all; clear; 
T = 0.1;
ITER = 1e3; %number of iterations

% define the system: 
statetransition_f = @state_function; 
measurement_f = @measurement_function; 
state_j = @state_jacobian; 
measurement_j = @measurement_jacobian; 

% define signal parameters: system
var_v1 =1e-1; 
var_v2 =1e-1; 
var_v3 = 1e-1; 
state_covariance = diag([var_v1 var_v2 var_v3]); 


% define signal parameters: observation/sensor
var_w1 = 1e-2; 
var_w2 = 1e-2; 
measurement_covariance = diag([var_w1 var_w2]); 

initial_x = [0; 0; 0]; %Last optimal predicted value (X_hat{k-1}): zero initially 

%define EKfilter object 
filter = ExtendedKF(statetransition_f, measurement_f, state_j, measurement_j, state_covariance,...
              measurement_covariance, T, initial_x, true)
for k = 1:ITER   
    [Xpred, Ppred] = filter.predict(); 
    [Xcorr, Pcorr] = filter.correct(); 
end

figure()
hold on 
plot(filter.truehistory(1, 2:end), filter.truehistory(2, 2:end), 'black', 'linewidth', 2)
plot(filter.predhistory(1, 2:end), filter.predhistory(2, 2:end), 'red--', 'linewidth', 2)
% plot(filter.measurementhistory(1, 2:end).*cos(filter.measurementhistory(2, 2:end)),...
%     filter.measurementhistory(1, 2:end).*sin(filter.measurementhistory(2, 2:end)), 'o')
title("Trajectory of Non-Linear system", 'fontsize',14)
lgd = legend('True trajectory','Estimated trajectory', 'Measurement','location', 'best')
lgd.FontSize = 12; 
hold off
print -depsc results.eps