function [D,Z,W,U]=processUponObservations(A,D,Z,Q,M,ind)
% generating process noise
W=[0;0;0;sqrt(Q(4,4))*randn(1);sqrt(Q(5,5))*randn(1);sqrt(Q(6,6))*randn(1)]; 
% generating observation noise
U=[sqrt(M(1,1))*randn(1);sqrt(M(1,1))*randn(1);sqrt(M(1,1))*randn(1)]; 
% State process
D=A*D+W; 
% Argument
ARG=AngleUnwrap(D(2),D(1),ind); 
 % observation
 Z=[sqrt(D(1)^2+D(2)^2);ARG;D(3)]+U; 
end