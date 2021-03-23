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
timespan = 100.0;  % simulation time in sec
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


%% Getting data from rosbag
bag_name = "2021-03-22-16-22-45.bag";    
bag = rosbag(bag_name);
gt_bag_topic = select(bag,'Topic','/simulator/module/ground_truth/pose');
noisy_bag_topic = select(bag,'Topic','/simulator/module/noisy/pose');
gt_msgStructs = readMessages(gt_bag_topic,'DataFormat','struct');
noisy_msgStructs = readMessages(noisy_bag_topic,'DataFormat','struct');

len = size(noisy_msgStructs);
len2 = len(1);
measurement = zeros(len2, 4);   %[x ; y ; z ; pitch]
measurement(:,1)= cellfun(@(m) double(m.Pose.Position.X),noisy_msgStructs)-0.2;
measurement(:,2)= cellfun(@(m) double(m.Pose.Position.Y),noisy_msgStructs)+10;
measurement(:,3)= cellfun(@(m) double(m.Pose.Position.Z),noisy_msgStructs);

quat = zeros(len2, 4);   %[x ; y ; z ; w]
quat(:,1)= cellfun(@(m) double(m.Pose.Orientation.X),noisy_msgStructs);
quat(:,2)= cellfun(@(m) double(m.Pose.Orientation.Y),noisy_msgStructs);
quat(:,3)= cellfun(@(m) double(m.Pose.Orientation.Z),noisy_msgStructs);
quat(:,4)= cellfun(@(m) double(m.Pose.Orientation.W),noisy_msgStructs);
eul = quat2eul(quat(:,:)); %default z,y,x
measurement(:,4) = -eul(:,2);


len = size(gt_msgStructs);
len1 = len(1);
real_pose = zeros(len1, 4);   %[x ; y ; z ; pitch]
real_pose(:,1)= cellfun(@(m) double(m.Pose.Position.X),gt_msgStructs) -0.2;
real_pose(:,2)= cellfun(@(m) double(m.Pose.Position.Y),gt_msgStructs) +10;
real_pose(:,3)= cellfun(@(m) double(m.Pose.Position.Z),gt_msgStructs);

quat = zeros(len1, 4);   %[x ; y ; z ; w]
quat(:,1)= cellfun(@(m) double(m.Pose.Orientation.X),gt_msgStructs);
quat(:,2)= cellfun(@(m) double(m.Pose.Orientation.Y),gt_msgStructs);
quat(:,3)= cellfun(@(m) double(m.Pose.Orientation.Z),gt_msgStructs);
quat(:,4)= cellfun(@(m) double(m.Pose.Orientation.W),gt_msgStructs);
eul = quat2eul(quat(:,:)); %default z,y,x
real_pose(:,4)=-eul(:,2);

len = min(len1, len2);

real_state = zeros(len, 6);
real_state(:,1) = -eul(1:len,2);
real_state(:,2) =  eul(1:len,3);
real_state(:,3) = -(real_state(:,1)-circshift(real_state(:,1), [-1 0]))*fs;
real_state(:,4) = -(real_state(:,2)-circshift(real_state(:,2), [-1 0]))*fs;

real_state(:,5) = 0.1;
real_state(:,6) = 2;

%% testing if extraction worked
% module_pos_gt = zeros(len, 3); %x, y, z
% module_pos_gt(:,1) = L_mast_gt*sin(-real_state(:,2));
% module_pos_gt(:,2) = L_mast_gt*sin(real_state(:,1));
% module_pos_gt(:,3) = L_mast_gt*cos(real_state(:,1)).*cos(real_state(:,2));
% 
% figure;
% hold on;
% plot(module_pos_gt(:,3),'b-');
% plot(measurement(:,3),'r+');

%% Initialisation of the Kalman filter
% ekf functions
f = @f_kf;
h = @h_kf;
del_f = @delta_f_kf;
del_h = @delta_h_kf;
R = diag(noise_std.^2)/3; % sensor noise
Q = diag([0.02, 0.02, 0.04, 0.04, ... %pitch, roll, pitch', roll'
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
X_save = zeros(len, N_STATE);
X_save(1,:) = X(:);
Xp_save = zeros(len, N_STATE);
Xp_save(1,:) = X(:);

% The error covariance
P = Q;
P(5,5) = 0.1;
P(6,6) = 0.1;
P_save = zeros(len, N_STATE);


%% Kalman in action
for run = 2 : len
    % Prediction
    %Xp = f(real_state(run,:));    
    Xp = f(X);
    Pp = del_f(X)*P*del_f(X)'+Q;
    
    if run<len*3/4 || not(prediction)
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

module_pos_estimate = zeros(len,4);
module_pos_estimate(:,1) = X_save(:,6).*sin(X_save(:,1));
module_pos_estimate(:,2) = X_save(:,6).*sin(X_save(:,2));
module_pos_estimate(:,3) = X_save(:,6).*cos(X_save(:,1)).*cos(X_save(:,2));
module_pos_estimate(:,4) = X_save(:,4);

%% Plotting
t=(0:len-1);
labels = ["x","y","z","pitch"];
%plotting x, y and z position of the module on different graphs
figure;
for i = 1:4
    subplot(2,2,i);
    hold on;
    plot(t,measurement(1:len,i),'r+');
    plot(t,real_pose(1:len,i),'k');
    plot(t,module_pos_estimate(:,i),'g');
    plot(t,Xp_save(:,i),'b.');
    %xline(timespan*3/4);
    xlabel('time');ylabel(labels(i));
end
%plot(t,X_save(:,3),'b.');
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
% /!\ can't be ploted anymore as I don't know the sim specs
% figure;
% labels2 = ["wave frequency","mast length"];
% X_save(:,5) = X_save(:,5)/2/pi;
% real_state(:,5) = real_state(:,5)/2/pi;
% for i = 5:6
%     subplot(1,3,i-4);
%     hold on;
%     plot(t,real_state(1:len,i),'k');
%     plot(t,X_save(:,i),'g');
%     xlabel('time');ylabel(labels2(i-4));
% end
% legend('real', 'estimated');
% 
% Ptrace = zeros(len);
% for i = 1:len
%     Ptrace(i) = sum(P_save(i,:));
% end
% %subplot(1,3,3);
% hold on;
% % plot(t,Ptrace);
% plot(t,P_save(1:len,5));
% plot(t,P_save(1:len,6));
% xlabel('time');ylabel("covariance");
% legend('wave frequency','mast length');
% % legend('trace', 'wave frequency','mast length');




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