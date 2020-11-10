function   [Xh,P]=predictionEstimateErrorCovariance(A,Xh,P,Q)
% Estimation
Xh=A*Xh;
% Priory Error Co-variance
P=A*P*A'+Q;
end