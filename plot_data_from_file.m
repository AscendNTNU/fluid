%% This script aim to read a log text file and display the data in a nice way with matlab
% In can be done online on ubuntu through https://matlab.mathworks.com/

%% A logfile should be shaped with a line of title in the first line,
% and then the raw data. One could also print the name of a varial each
% line, witht he sequence 'variable_name \t data' as many times as we have
% variable to print.
% For exemple, some logfiles are generated by extract_module_operation.cpp
% and model_position_publisher.py


clear();
fid = fopen('log_drone_pos_and_velocity.txt'); %open the datafile
% log_module_position.txt
% log_drone_pos_and_velocity.txt
firstLine = fgets(fid);     %to later find the title of each column
firstLine = textscan(firstLine,'%s');
L = size(firstLine{1});     % to find the number of columns
L = L(1);
dataFormat = "";
for i = 1:L
    dataFormat=dataFormat+"%s";
end


%% Analyse the text file to get its format 
titleInColumn = 1;  %variable title reprint each line
for i=1:L
    if size(str2num(char(firstLine{1}{i})))==1
        titleInColumn=2; %lines only get data (IE. the title is only print on the first line
    end
end


titles = strings(L/titleInColumn,1);    %The name of each variable
for i = 1:titleInColumn:L
    titles(i) = firstLine{1}{i};
end

%% for readability: 
titles = strrep(titles,"_","\_");
titles = strrep(titles,"�","ation");
if size(char(titles(1)))==[1 1] titles(1) = "time"; end
if size(char(titles(2)))==[1 1] titles(2) = "position X"; end
if size(char(titles(3)))==[1 1] titles(3) = "position Y"; end

%% We finally get the data we are looking for
C = textscan(fid, dataFormat,'Headerlines',2-titleInColumn);   %return a table with all our data
fclose(fid);
N = size(C{end}); % number of line in the table
N = N(1);
results = zeros(L,N); %translate to a matrix
for i = titleInColumn:titleInColumn:L
    for j=1:N
        a = C{i}{j};
        a = str2double(a);
        results(i,j) = a(1);
    end
end

%% Here we try to lay all the graps out so that they nicely fit on our screen
% So we wonder how many lines and columns should we use to display all the
% graphs.
if isprime(L/titleInColumn) && L/titleInColumn > 3
    plotSize = factor(L/titleInColumn+1);
else
    plotSize = factor(L/titleInColumn);
end
if L/titleInColumn <=3
    plotSize = L/titleInColumn;
end
nbfactor =size(plotSize);
while nbfactor(2)>=3
    plotSize = [plotSize(1)*plotSize(2) plotSize(3:end)];
    nbfactor =size(plotSize);
end
plotSize = sort(plotSize);
if size(plotSize)==[1 1]
    plotSize = [1 plotSize];
end



%% display results
figure
for i = 2*titleInColumn:titleInColumn:L
    subplot(plotSize(1),plotSize(2),i/titleInColumn);   %,'replace')
    plot(results(titleInColumn,:),results(i,:));
    ylabel(titles(i/titleInColumn));
    xlabel(titles(1));
end
% we also want to print y against x to see the position of the robot
subplot(plotSize(1),plotSize(2),1)
plot(results(2*titleInColumn,:),results(3*titleInColumn,:));
xlabel(titles(2));
ylabel(titles(3));

%% Other fewer display to get them bigger
figure
subplot(1,3,1)
plot(results(2*titleInColumn,:),results(3*titleInColumn,:));
xlabel(titles(2));
ylabel(titles(3));
for i = 2*titleInColumn:titleInColumn:3*titleInColumn
    subplot(1,3,i/titleInColumn);   %,'replace')
    plot(results(titleInColumn,:),results(i,:));
    ylabel(titles(i/titleInColumn));
    xlabel(titles(1));
end

