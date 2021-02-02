function [samples,delay] = time_and_distance_error_f(reference,measurement)
%time_and_distance_error_f This function measure the shift in time and
%value between the reference and the measurement.
% This algorith suppose that the two times series have the same sample time,
% and have the time in their first column and the value in the second one.
reference = reshape(reference,2,[]);
measurement = reshape(measurement,2,[]);


% zci returns Zero-Crossing Indices Of Argument Vector
zci = @(v) find(v(:).*circshift(v(:), [-1 0]) <= 0);



%% Make sure that they start at the same time.
if reference(1,2) < measurement(1,1)
    i = 1;
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

%% Find main frequency of the reference
% This is needed to estimate when does the steady state is reached.
% The transition periode will be concidered to last half a period
% I am not using the fft method because the frequency of the signal is
% quite low compare to the length of it.

Fs=length(reference(1,:))/(reference(1,end)-reference(1,1));
crossing_reference_indexes = zci(reference(2,:));
if length(crossing_reference_indexes)<=4
    fprintf('/!\\ The sample is not long enough to estimate errors! /!\\ \n'); %todo, try to write it in red
    return
elseif crossing_reference_indexes(2) - crossing_reference_indexes(1) == 1 %here we have zeros in the ref and so zci returns each crossing zero twice
    crossing_reference_indexes = crossing_reference_indexes(1:2:end);
end
sample_per_sin = 2*(circshift(crossing_reference_indexes,[-1 0]) - crossing_reference_indexes);
sample_per_sin = mean(sample_per_sin(2:end-2));
period = sample_per_sin/Fs;
start = round(sample_per_sin/2);
%fprintf("%2.3fsec will be omited for calulculation, which correspond" + ...
%                "to the %i/%i element\n",period/2,start,length(reference));


%% Error calculation
% for each measurement point, I look for the closest point of reference at
% the same time.
error = zeros(2,sample_length-start+1);
error(1,:) = reference(1,start:end);
error(2,:) = measurement(2,start:end) - reference(2,start:end);

%% Timeshift calculation
% for each measurement point, I look for the first point in the past
% (within some boundaries I guess) that has about the same value.
delay = finddelay(reference(2,start:end),measurement(2,start:end)) /Fs ;
time_step = (reference(1,end)-reference(1,1))/length(reference);
delay2 = finddelay(reference(2,:),measurement(2,:)) * time_step;

%% Average maximum error calculation
% The maximum error of each half period will be calculated and averaged

first_zero_ind = crossing_reference_indexes(2);
round_half_sample_periode = round(sample_per_sin/2);
last_zero_ind =  first_zero_ind+floor((length(measurement)-first_zero_ind+1)/round_half_sample_periode)*round_half_sample_periode;
samples = reshape(error(2,first_zero_ind-start:last_zero_ind-start-1),round_half_sample_periode,[]);

%% Angle of the maximum error calculation
% we want to know where is the module when the drone has its biggest error
% The angle is between 0 and 180° as I consider the angle of sin signal.

[maxes,i] = max(samples);

%% print some information

%fprintf("average time step: \t\t%.3f s\n",time_step);
fprintf("average delay: \t\t\t %.3f s\n",delay);
fprintf("average absolute error: \t %.3f m\n",sum(abs(error(2,:)))/length(error));
fprintf("average max absolute error: \t %.3f m\n",mean(max(abs(samples))));
fprintf("max absolute error: \t %.3f m\n",max(abs(error(2,:))));

end
% todo: do it for 2D array (i.e. taking into account both x and y)
