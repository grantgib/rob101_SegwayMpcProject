function [u] = Extract_Torques(t,x,params)
u = zeros(length(t),1);
for i = 1:length(t)
    [~,u(i)] = Segway_ode(t(i),x(i,:)',params);
end

end