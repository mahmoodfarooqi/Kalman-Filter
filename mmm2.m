%%%% Q. Thermal camera installed on an Airborne platform observed position of a stationary target.Thus Obtained
%%% position is noisy because Thermal camera is affecting certain vibration due to malfunctioned platform. The
%%% Objective is to estimate the position (along x-axis) of target. The measurement noise is additive having Gaussian
%%% PDF with zero mean and standard deviation of 1 m. Observation time is 5 seconds and sampling time is 0.1 second.
%%************************************************************************%%%
%%%                                  Steps                                %%%
%%% 1.Global variable delaration and common variables                     %%%
%%% 2. Kalman Filter                                                      %%%
%%%   2.1. The prediction part                                            %%%
%%%   2.2. The estimation Part                                            %%%                                                           %%%
%%% 3. Averaging the results to eleminate uncertainity (MonteCarlo Runs)  %%%
%%% 4. Statistics (Root mean Sauare Error)                                %%%
%%% 5. Plotting                                                           %%%
%%%***********************************************************************%%%
clc
close all
clear all
T = 0.1;                     %sampling time
total_time = 5;
t =0: T: total_time;        %time vector
mea=1;      % No of meaurements
I=eye(mea);                       %identity Matrix
mean = 0;                    %measurement noise mean
meas_noise_sd = 5; % measurement noise standard deviation
init_pos = [110];    %inital Position x
F=I; % Transition matrix
H=I;   %Output Coeffecient Matrix 
R=meas_noise_sd^2*[I]    % measurement covariance matrix
for m=1:50 % for 50 runs
Ztrue=init_pos*ones(1,length(t))
meas_noise = mean +meas_noise_sd*randn(1,length(t));    % generating measurement noise (gaussian noise) 
 Znoisy = Ztrue + meas_noise;                     % The noisy measurment vector

 X0 = [Znoisy(:,1)];                   % inital State  
 P0 = R*[I];                  % inital State Covariacne
 targetX = Ztrue;
    for cycle = 1 :length(t)

        if cycle==1
           X_est = X0;         % inital State
            P_est = P0;       % inital State covariance
        else
X_pri=F*X_est;       % Predicted state vector
P_pri=F*P_est*F' ;    % Predicted state covariance matrix
S = H*P_pri*H' + R;     % Innovation covariance
K = P_pri*H'*inv(S);    % optimal kalman gain

X_est = X_pri + K*( Znoisy(1,cycle) - H*X_pri );    % state estimate
P_est = ( I - K*H )*P_pri;
        end
        Est_X(m,cycle) = X_est;
    end
    Znoisyx(m,:)=Znoisy;
end
M_estX = sum(Est_X)/50; % Finding means of all runs
M_Znoisyx = sum(Znoisyx)/50;
% M_estX =(EstimatedX);
% M_Znoisyx=Znoisyx
RMSE = sqrt( sum( (repmat(Ztrue(1,:),50,1) - Est_X).^2 )/50 ); % Root Mean Square error
%%%%%%%%%% ploting
 plot(t,Ztrue,'r','LineWidth',1.5)
 hold on
 plot(t,Znoisy,'g','LineWidth',1.5)
  hold on
  plot(t,M_estX,'b','LineWidth',1.5)
  legend('True Position','Measured Position','Estimated Position')
   xlabel('Time in seconds')
    ylabel('Position in meters')
    title('Target position')
    grid on
  figure
  plot(t,RMSE,'LineWidth',1.5)
   xlabel('Time in seconds')
    ylabel('RMSE in meter')
    title('Target Root Mean Square Error')
    grid on