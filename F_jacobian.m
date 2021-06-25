function F_k = F_jacobian(T, vk, xhat_last)
%Computes/updates jacobian matrix
phi_last = xhat_last(3); 

F_k = [1 0 -T*vk*sin(phi_last); 0 1 T*vk*cos(phi_last); 0 0 1]; 

end

