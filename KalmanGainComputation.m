function   K=KalmanGainComputation(H,P,M)
K=P*H'*(M+H*P*H')^(-1);
end