
function [ risk ] = collisionRiskFunction( d )
%COLLISIONRISKFUNCTION Summary of this function goes here
%   Detailed explanation goes here

rho = 0.4;
alpha = 6;

risk = 1 / (1 + exp( (d*(2/rho)-1)*alpha) );

end

