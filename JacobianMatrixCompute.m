% Computation Of Jacobian Matrix 
function  H=JacobianMatrixCompute(X,Y,Z)
H=[X/(sqrt(X^2+Y^2)), Y/(sqrt(X^2+Y^2)),0,0,0,0;
   -Y/(sqrt(X^2+Y^2)), X/(sqrt(X^2+Y^2)),0,0,0,0;
   0,0,1,0,0,0]; 
end