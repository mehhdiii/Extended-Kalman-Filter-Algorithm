
% define the system: 
vk = 0.1; 
wk = 0.01;
T = 0.05;
% xk = f(xk-1)+vk = xk-1 + Tvkcos(phi{k-1}); yk-1 + Tvk sin(phi{k-1});
% phi{k-1} + T*wk




% define signal parameters: system
var_v1 =1e-6; 
var_v2 =1e-6; 
var_v3 = 1e-6; 
Q_k = diag([var_v1 var_v2 var_v3]); 


% define signal parameters: observation/sensor
var_w1 = 1e-4;
var_w2 = 1e-4; 
W_k = diag([var_w1 var_w2]); 

ITER = 1e3; %number of iterations
x_last = zeros(3,1); %last value of states (X{k-1}): zero initially 
xhat_last = zeros(3,1); %Last optimal predicted value (X_hat{k-1}): zero initially 
P_last = eye(3); % Last covariance matrix value for the estimated states

y_last = 0; %last observation: zero initially 

%storage of values 
historyX_k = [0; 0; 0]; 
historyY_k = [0; 0];
historyX_predict = [0; 0; 0]; 

for k = 1:ITER   
   
    %generate noise values: 
    v = sqrt(Q_k)*randn(3, 1); 
    w = sqrt(W_k)*randn(2, 1); 
    
    %generate xk and yk: 
    x_k = [
        x_last(1) + T*vk*cos(x_last(3)); 
        x_last(2)+T*vk*sin(x_last(3)); 
        x_last(3)+T*wk; 
        ] + v; 
    
    y_k = [sqrt(x_k(1)^2+x_k(2)^2); atan(x_k(2)/x_k(1))] + w;
    
    %run KF algorithm 
    [xhat_optimal,P_optimal] = KalmanFilter(y_k, Q_k, W_k, xhat_last, P_last, vk, wk, T); 
    xhat_last = xhat_optimal; 
    y_last = y_k; 
    x_last = x_k; 
    P_last = P_optimal; 
    historyX_k(:, k) = x_k; 
    historyY_k(:, k) = y_k; 
    historyX_predict(:, k) = xhat_optimal; 
end

P = 50
figure()
hold on 

plot(historyX_k(1, end-P:end), historyX_k(2, end-P:end), 'k-')
plot(historyY_k(1, end-P:end).*cos(historyY_k(2, end-P:end)), historyY_k(1, end-P:end).*sin(historyY_k(2, end-P:end)), 'rx')
plot(historyX_predict(1, end-P:end), historyX_predict(2, end-P:end), 'b--')
xlabel("x coordinate", 'fontsize',12)
ylabel("y coordinate", 'fontsize',12)
title("Trajectory of Non-Linear system", 'fontsize',14)
lgd = legend('True trajectory', 'Sensor measurement', 'Predicted trajectory', 'location', 'best')
lgd.FontSize = 14
hold off
print -depsc results.eps