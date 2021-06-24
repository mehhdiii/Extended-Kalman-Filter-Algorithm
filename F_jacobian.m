function Fk = F_jacobian(T, vk, phi_last)
%Computes/updates jacobian matrix

Fk = [1 0 -T*vk*sin(phi_last); 0 1 T*vk*cos(phi_last); 0 0 1]

end

