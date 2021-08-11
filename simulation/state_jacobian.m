function F_k = state_jacobian(xhat_last, T)
%Computes/updates jacobian matrix
phi_last = xhat_last(3); 
vk = 0.1;
F_k = [1 0 -T*vk*sin(phi_last); 0 1 T*vk*cos(phi_last); 0 0 1]; 

end

