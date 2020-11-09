function [error,delay] = time_and_distance_error_f(reference,measurement)
%time_and_distance_error_f This function measure the shift in time and
%value between the reference and the measurement.
% This algorith suppose that the two times series have the same sample time,
% and have the time in their first column and the value in the second one.
reference = reshape(reference,2,[]);
measurement = reshape(measurement,2,[]);




%% Make sure that they start at the same time.
if reference(1,2) < measurement(1,1)
    i = 3;
    while reference(1,i) < measurement(1,1)
        i=i+1;
    end
    % check if the time before was not closer to the measurment
    if reference(1,i)-measurement(1,1) > measurement(1,1) - reference(1,i-1) 
        i=i-1;
    end
    reference = reference(:,i:end);
end

if measurement(1,2) < reference(1,1)
    i = 3;
    while measurement(1,i) < reference(1,1)
        i=i+1;
    end
    % check if the time before was not closer to the measurment
    if measurement(1,i)-reference(1,1) > reference(1,1) - measurement(1,i-1) 
        i=i-1;
    end
    measurement = measurement(:,i:end);
end

sample_length = min(sum(reference(1,:)>0),sum(measurement(1,:)>0));
reference = reference(:,1:sample_length);
measurement = measurement(:,1:sample_length);

%% Error calculation
% for each measurement point, I look for the closest point of reference at
% the same time.
error = zeros(2,sample_length);
error(1,:) = reference(1,:);
error(2,:) = measurement(2,:) - reference(2,:);

%% Timeshift calculation
% for each measurement point, I look for the first point in the past
% (within some boundaries I guess) that has about the same value.
time_step = (reference(1,end)-reference(1,1))/length(reference);
delay = finddelay(reference(2,:),measurement(2,:)) * time_step;

%% print some information

%fprintf("average time step: \t\t%.3f s\n",time_step);
fprintf("average delay: \t\t\t %.3f s\n",delay);
fprintf("average absolut error: \t %.3f m\n",sum(abs(error(2,:)))/sample_length);
fprintf("max absolute error: \t %.3f m\n",max(abs(error(2,:))));

end
% todo: do it for 2D array (i.e. taking into account both x and y)
