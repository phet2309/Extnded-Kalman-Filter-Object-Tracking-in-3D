%
%KALMAN FITLER PROJECT
%estimating trajectory of an object in 3-D
function ExtendedKalmanFilter
    clear all;
    close all;
    clc;
    
    
    % Measurement Noise matrix
    % This model includes measurement noise, Covarience matrix of measurment noise 
    Mes_Noise=[0.001 0 0;
               0 0.001 0;
               0 0 0.001]; 
    
    % Process Noise matrix
    % This model includes process Noise, Covarience matrix of process noise
    % noise
    Cov_Mat=[0 0 0 0 0 0;
            0 0 0 0 0 0;
            0 0 0 0 0 0;
            0 0 0 0.02 0 0;
            0 0 0 0 0.02 0;
            0 0 0 0 0 0.02]; 
    
    % Time for Sampling
    samp_time=0.1;
    
    % System Dynamics 
    system_dynamic=[1 0 0 samp_time 0 0;
                    0 1 0 0 samp_time 0;
                    0 0 1 0 0 samp_time;
                    0 0 0 1 0 0;
                    0 0 0 0 1 0;
                    0 0 0 0 0 1]; 
    
    % Actual initial conditions 
    % [x,y,z,vx,vy,vz]
    X(:,1)=[1;1;1;2;2;2]; 
    % initial observation
    Z(:,1)=[3;3;3]; 
    % Initial condition as per assumption
    Xh(:,1)=[1.6;1.6;1.6;20;20;20];
    %inital value of covarience of estimation error
    Est_Err(:,:,1)=[0.1 0 0 0 0 0;
                  0 0.1 0 0 0 0;
                  0 0 0.1 0 0 0;
                  0 0 0 0.1 0 0;
                  0 0 0 0 0.1 0;
                  0 0 0 0 0 0.1]; 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%     
    % Plott setup 
    
    figure(1)
    xlabel('time') 
    ylabel('X')
    title('X possition')
    grid on
    hold on
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    figure(2)
    xlabel('time') 
    ylabel('Y')
    title('Y possition')
    grid on
    hold on
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    figure(3)
    xlabel('time') 
    ylabel('Z')
    title('Z possition')
    grid on
    hold on
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    figure(4)
    xlabel('time') 
    title('Minimum MSE')
    grid on
    hold on
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    figure(5)
    plot3(0,0,0)
    title('3-D trajectory ')
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    grid on
    hold on
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % indicator function. Used for unwrapping of tan
    ind=0;
    for n=1:200
        % process noise generation, observation nois generation
        % State process, argument and observation
        [X(:,n+1),Z(:,n+1),w,u]=processUponObservations(system_dynamic,X(:,n),Z(:,n),Cov_Mat,Mes_Noise,ind);
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Plotting the x value according to observation made in
        % processUponObservation function
        figure(1)
        line([n,n+1],[X(1,n),X(1,n+1)])  
        hold on 
        drawnow
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Plotting the y value according to observation made in
        % processUponObservation function
        figure(2)
        line([n,n+1],[X(2,n),X(2,n+1)])  
        hold on
        drawnow
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Plotting the z value according to observation made in
        % processUponObservation function
        figure(3)
        line([n,n+1],[X(3,n),X(3,n+1)]) 
        hold on 
        drawnow
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Equations used for prediction
        % Estimation
        % Priory Error Co-variance
        %   prediction of next state
        [Xh(:,n+1),Est_Err(:,:,n+1)]=predictionEstimateErrorCovariance(system_dynamic,Xh(:,n),Est_Err(:,:,n),Cov_Mat); 
         
        % Correction Equations
          
        % Computation of Jacobian Matrix, for transformation
        H(:,:,n+1)=JacobianMatrixCompute(Xh(1,n+1),Xh(2,n+1),Xh(3,n+1));
        % Kalman Gain Computation
        K(:,:,n+1)=KalmanGainComputation(H(:,:,n+1),Est_Err(:,:,n+1),Mes_Noise); 
        % Innovation Computation
        % Non-linear Mapping
        Inov=InovationComputation(Z(:,n+1),Xh(:,n+1),ind); 
        % Final estimation Computation
        Xh(:,n+1)=Xh(:,n+1)+ K(:,:,n+1)*Inov; 
        % Covarience of estimation error
        Est_Err(:,:,n+1)=(eye(6)-K(:,:,n+1)*H(:,:,n+1))*Est_Err(:,:,n+1);
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        figure(5)
        % 3D plot of estimated trajectory 
        line([Xh(1,n) Xh(1,n+1)],[Xh(2,n) Xh(2,n+1)],[Xh(3,n) Xh(3,n+1)],'Color','r')
        hold on 
        drawnow
        % 3D plot of actual trajectory
        line([X(1,n) X(1,n+1)],[X(2,n) X(2,n+1)],[X(3,n) X(3,n+1)]) 
        hold on 
        drawnow
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        figure(4)
        % MSE
        line([n,n+1],[(X(1,n)-Xh(1,n))^2,(X(1,n+1)-Xh(1,n+1))^2]) 
        hold on
        drawnow
        % MSE
        line([n,n+1],[(X(2,n)-Xh(2,n))^2,(X(2,n+1)-Xh(2,n+1))^2],'Color','r') 
        hold on
        drawnow
        % MSE
        line([n,n+1],[(X(3,n)-Xh(3,n))^2,(X(3,n+1)-Xh(3,n+1))^2],'Color','c') 
        hold on
        drawnow
        legend('Square erro in X direction','Square erro in Y direction','Square erro in Z direction')
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        figure(1)
        % plotting estimated trajectory for x coordinate
        line([n,n+1],[Xh(1,n),Xh(1,n+1)],'Color','r') 
        hold on 
        drawnow 
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        figure(2)
        %plotting estimated trajectory for y coordinate
        line([n,n+1],[Xh(2,n),Xh(2,n+1)],'Color','r')
        hold on 
        drawnow 
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        figure(3)
        %plotting estimated trajectory for z coordinate
        line([n,n+1],[Xh(3,n),Xh(3,n+1)],'Color','r') 
        hold on 
        drawnow 
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % Unwrapping of tan
        theta1=AngleUnwrap(Xh(1,n+1),Xh(2,n+1),0); 
        theta=AngleUnwrap(Xh(1,n),Xh(2,n),0);
        
        if abs(theta1-theta)>=pi
            if ind==1
                ind=0    
        else
            ind=1
            end
        end
    
    end
end