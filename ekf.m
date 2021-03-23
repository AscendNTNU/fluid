%% Design of a non linear kalman filter for Ascend. 
% We are planning on beginning with an extended kalman filter, but other
% type of filter may be better.
% The sensors / measurements are x, y, z, pitch, yaw. The yaw is not fixed
% and therefore not useful for this kalman filter.

% In this simulation, we have x pointing foward, y pointing to the right
% and z upward

%% some simulation settings:
global fs N_STATE
N_STATE = 6;
fs = 25.0;        % sample frequency in Hz
timespan = 30.0;  % simulation time in sec
t=(0:1/fs:timespan-1/fs);
prediction = false; %stop using the measurement after 75% of the simulation

% Oscillations:
L_mast_gt = 2.0;     %Length of the mast in meters
%pitch aplitude in degrees:
Ap_gt = deg2rad(15.0) * ( 1 + sin(t/10+2*pi*rand())/5); 
%roll  aplitude in degrees:
Ar_gt = deg2rad(10.0) * ( 1 + sin(t/10+2*pi*rand())/5); 
% wave/ oscillation frequency in sec:
f_wave_gt = 0.1 * ( 1 + sin(t/10+2*pi*rand())/5);   
%relative phase between pitch and roll:
phase_gt = pi/5 * ( 1 + sin(t/10+2*pi*rand())/5);   

% Sensor noise (standard deviation)
noise_std = [0.0707
             0.0707
             0.0707
             0.022361];
         
% Example of a potential real state vector
real_state = zeros(timespan*fs, N_STATE); % [pitch ; roll]
real_state(:,1) = Ap_gt .* sin(f_wave_gt*2*pi.*t);
real_state(:,2) = Ar_gt .* sin(f_wave_gt*2*pi.*t+phase_gt);
real_state(:,3) = Ap_gt .* 2*pi.*f_wave_gt .* cos(f_wave_gt*2*pi.*t);
real_state(:,4) = Ar_gt .* 2*pi.*f_wave_gt .* cos(f_wave_gt*2*pi.*t+phase_gt);
real_state(:,5) = f_wave_gt*2*pi;
real_state(:,6) = L_mast_gt*ones(1,timespan*fs);

module_pos_gt = zeros(timespan*fs, 4); %x, y, z
module_pos_gt(:,1) = L_mast_gt*sin(real_state(:,1));
module_pos_gt(:,2) = L_mast_gt*sin(real_state(:,2));
module_pos_gt(:,3) = L_mast_gt*cos(real_state(:,1)).*cos(real_state(:,2));
module_pos_gt(:,4) = real_state(:,1);

% Creation of potential measurement from perception
measurement = zeros(timespan * fs, 4);   %[x ; y ; z ; pitch]
measurement(:,1) = module_pos_gt(:,1) + noise_std(1)*randn(timespan*fs, 1);
measurement(:,2) = module_pos_gt(:,2) + noise_std(2)*randn(timespan*fs, 1);
measurement(:,3) = module_pos_gt(:,3) + noise_std(3)*randn(timespan*fs, 1);
measurement(:,4) = real_state(:,1)    + noise_std(4)*randn(timespan*fs, 1);

%% Initialisation of the Kalman filter
% ekf functions
f = @f_kf;
h = @h_kf;
del_f = @delta_f_kf;
del_h = @delta_h_kf;
R = diag(noise_std.^2)/3; % sensor noise
Q = diag([0.005, 0.005, 0.04, 0.04, ... %pitch, roll, pitch', roll'
          0.001, 0.0005]); % model error , omega, L_mast
     

Q = Q*0.01; 
R = R*10;
%R = 0.01*eye(4);
%Q = 0.01*eye(N_STATE);

X = zeros(N_STATE,1); %pitch, roll, pitch', roll', omega, L_mast
X(1:4) = measurement(1,1:4);
X(5) = 1;
X(6) = 1.9;
%X(:) = real_state(1,:);

% Save of X for plotting
X_save = zeros(timespan * fs, N_STATE);
X_save(1,:) = X(:);
Xp_save = zeros(timespan * fs, N_STATE);
Xp_save(1,:) = X(:);

% The error covariance
P = Q;
P(5,5) = 0.1;
P(6,6) = 0.1;
P_save = zeros(timespan * fs, N_STATE);


%% Kalman in action
for run = 2 : timespan*fs
    % Prediction
    Xp = f(X);    
    Pp = del_f(X)*P*del_f(X)'+Q;
    
    if run<timespan*fs*3/4 || not(prediction)
        % Update
        K = Pp*del_h(X)'/(R+del_h(X)*Pp*del_h(X)');
        P = Pp - K*del_h(X)*Pp;
        X = Xp + K*(measurement(run, :)'-h(Xp));
    else
        X = Xp;
        P = Pp;
    end
    X_save(run,:) = X(:);
    Xp_save(run,:) = Xp(:);
    P_save(run,:) = diag(P);

end

module_pos_estimate = zeros(timespan*fs,4);
module_pos_estimate(:,1) = L_mast_gt*sin(X_save(:,1));
module_pos_estimate(:,2) = L_mast_gt*sin(X_save(:,2));
module_pos_estimate(:,3) = L_mast_gt*cos(X_save(:,1)).*cos(X_save(:,2));
module_pos_estimate(:,4) = X_save(:,1);

%% Plotting
t=(0:1/fs:timespan-1/fs);
labels = ["x","y","z","pitch"];
%plotting x, y and z position of the module on different graphs
figure;
for i = 1:4
    subplot(2,2,i);
    hold on;
    plot(t,measurement(:,i),'r+');
    plot(t,module_pos_gt(:,i),'k');
    plot(t,module_pos_estimate(:,i),'g');
    %plot(t,Xp_save(:,i),'b.');
    %xline(timespan*3/4);
    xlabel('time');ylabel(labels(i));
end
hold off;
legend('measurement','real postition','estimation');%,'prediction');

%x,y plot

% figure;
% hold on;
% plot(measurement(:,1),measurement(:,2),'r+');
% plot(module_pos_gt(:,1),module_pos_gt(:,2),'k');
% plot(module_pos_estimate(:,1),module_pos_estimate(:,2),'g');
% legend('measurement','real postition','estimation');%,'prediction');
% axis square;


% plot the frequency of the waves and the estimation
figure;
labels2 = ["wave frequency","mast length"];
X_save(:,5) = X_save(:,5)/2/pi;
real_state(:,5) = real_state(:,5)/2/pi;
for i = 5:6
    subplot(1,3,i-4);
    hold on;
    plot(t,real_state(:,i),'k');
    plot(t,X_save(:,i),'g');
    xlabel('time');ylabel(labels2(i-4));
end
legend('real', 'estimated');

Ptrace = zeros(fs*timespan);
for i = 1:fs*timespan
    Ptrace(i) = sum(P_save(i,:));
end
subplot(1,3,3);
hold on;
% plot(t,Ptrace);
plot(t,P_save(:,5));
plot(t,P_save(:,6));
xlabel('time');ylabel("covariance");
legend('wave frequency','mast length');
% legend('trace', 'wave frequency','mast length');




%% KF functions definition
%gradient of h function
function M = delta_h_kf(X)
    global N_STATE
    p = X(1);
    r = X(2);
    L = X(6);
    M = zeros(4,N_STATE);
    M(1,1) = L*cos(p);
    M(2,2) = L*cos(r);
    M(3,1) = -L*cos(r)*sin(p);
    M(3,2) = -L*cos(p)*sin(r);
    M(4,1) = 1;
    M(1,6) = sin(p);
    M(2,6) = sin(r);
    M(3,6) = cos(r)*cos(p);
    %M = M';
end


%gradient of f function
function M = delta_f_kf(X)
    global fs N_STATE
    p = X(1);
    r = X(2);
    w = X(5);
    M = eye(N_STATE);
    M(1,3) = 1/fs;
    M(2,4) = 1/fs;
    M(3,1) = - w^2/fs;
    M(4,2) = - w^2/fs;
    M(3,5) = -2*w*p/fs;
    M(4,5) = -2*w*r/fs;
end

% Measurement = h(k,X)
function Y = h_kf(X)
    p = X(1);
    r = X(2);
    L = X(6);   %length of the mast
    x = L * sin(p);
    y = L * sin(r);
    z = L * cos(p)*cos(r);
    pitch = p;
    Y = [x,y,z,pitch]';
end

% X_k+1  = f(X_k)
function X_n = f_kf(X)
    global fs
    p = X(1);
    r = X(2);
    w = X(5);
    p_dot = X(3);
    r_dot = X(4);
    p_n = p + p_dot/fs;
    r_n = r + r_dot/fs;
    p_dot_n = p_dot - w^2 * p /fs;
    r_dot_n = r_dot - w^2 * r /fs;
    X_n = [p_n, r_n, p_dot_n, r_dot_n, X(5), X(6)]';
end