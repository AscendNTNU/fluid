%% The aim of this script is to compare data from two different files.
% The way it does it is by finding similar titles and then plot the similar
% variables on the same graphs
% This code is written from plot_data_from_file.m

clear();
files_to_compare = [ % put the files you want to compare h
    "drone_pos_and_velocity.txt"
    "module_position.txt"
    ]';

nb_file = length(files_to_compare);
files = [];
first_lines = cell(nb_file,1);
nb_column = zeros(nb_file,1);
dataFormat = strings(nb_file,1);

for i = 1:nb_file %we will have to repeat all operation for every file
    files(i) = fopen(files_to_compare(i));
    %first_lines(i) = fgets(files(i));
    first_lines(i) = textscan(fgets(files(i)),'%s'); %translate into an array of "words"/numbers
    L = size(first_lines{i});
    nb_column(i) = L(1);                   % good type
    for j = 1:nb_column(i)
        dataFormat(i)=dataFormat(i)+"%s";
    end

    
    %% Analyse the text file to get its format 
    titleInColumn = ones(nb_file,1);  %variable title reprint each line
    for j=1:nb_column(i)
        if size(str2num(char(first_lines{i}{j})))==1
            titleInColumn(i)=2; %lines only get data (IE. the title is only print on the first line
        end
        
    end
end % here we have to end the for because we need to initialize new variables only once

%% We finally get the data we are looking for
% First the titles
titles = strings(nb_file,max(nb_column));    %The name of each variable    
results=[];
for i=1:nb_file
    for j = 1:titleInColumn(i):nb_column(i)
        titles(i,j) = first_lines{i}{j};%todo: check it works with the other format of file
    end
end

%% Let us compare the title and make a single list of them all
merge_titles = titles(1,1:nb_column(1));
data_fusion_matrix = zeros(size(titles)); %todo, fix the evalutation of this matrix when the first files is smaller than the second one
data_fusion_matrix(1,1:nb_column(1)) = 1:nb_column(1);
for i=2:nb_file
    for j=1:nb_column(i)
        pos = find(merge_titles==titles(i,j));
        if pos
            data_fusion_matrix(i,j)=pos;
        else
            merge_titles(length(merge_titles)+1) = titles(i,j);
            data_fusion_matrix(i,j)=length(merge_titles);
        end
    end
end

nb_lines = zeros(nb_file,1);
%% Then, we can find the data associated to the titles
for i=1:nb_file
    C = textscan(files(i), dataFormat(i),'Headerlines',2-titleInColumn(i));   %return a table with all our data
    fclose(files(i));
    N = size(C{end}); % number of line in the table, uses the last colomn as it may be shorter
    nb_lines(i) = N(1);
    results(i,nb_column(i),nb_lines(i)) = 0; %increase the size of the matrix if needed, I don't know a better way to do it...
    
    for j = titleInColumn:titleInColumn:nb_column(i)
        column = str2double(C{j});
        results(i,data_fusion_matrix(i,j)*titleInColumn(i),1:nb_lines(i)) = column; %we have to do it column by column because C is a made of cells, and I don't know  how to elsewise
    end
end
% TODO: check that if a latter file has more columns, all the titles are
% savede, and there is no error btw ^^

%TODO: test that elements are set in the good columns
%% shift the time if it does not begin near 0
start_time = min(results(:,1,1));
if start_time >1000
    for i=1:nb_file
        i
        results(i,1,1:nb_lines(i))=results(i,1,1:nb_lines(i))-start_time;
    end
end

%% for readability: 
for i=1:nb_file
    titles(i) = strrep(titles(i),"_","\_");
    titles(i) = strrep(titles(i),"°","ation");
    if size(char(titles(i,1)))==[1 1] titles(i,1) = "time"; end
    if size(char(titles(i,2)))==[1 1] titles(i,2) = "position X"; end
    if size(char(titles(i,3)))==[1 1] titles(i,3) = "position Y"; end    
end
files_to_compare = strrep(files_to_compare,"_"," ");
files_to_compare = strrep(files_to_compare,".txt","");
files_to_compare = strrep(files_to_compare,"log","");

%% Here we try to lay all the graps out so that they nicely fit on our screen
% So we wonder how many lines and columns should we use to display all the
% graphs.
L = length(merge_titles)+1; %the number of graph we are gonna display + legend
if isprime(L) && L > 3
    plotSize = factor(L+1);
else
    plotSize = factor(L);
end
if L <=3
    plotSize = L;
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
colors = lines(nb_file);
figure
L = length(merge_titles);
for i = 2*titleInColumn(1):titleInColumn(1):L %i is the variable we are gonna display
    subplot(plotSize(1),plotSize(2),i/titleInColumn(1));   %,'replace')
    for file_ind=1:nb_file
        if sum(merge_titles.contains(titles(file_ind,i))) && not(titles(file_ind,i)=="") % a log has data if it has the required title
            plot(shiftdim(results(file_ind,titleInColumn(1),1:nb_lines(file_ind)),1),shiftdim(results(file_ind,i,1:nb_lines(file_ind)),1),'color',colors(file_ind,:));
            hold on
        end
    end
    ylabel(merge_titles(i/titleInColumn(1)));
    xlabel(merge_titles(1));
    hold off
end

%legend(files_to_compare(1),files_to_compare(2);
% we also want to print y against x to see the position of the robot
subplot(plotSize(1),plotSize(2),1)
for file_ind=1:nb_file
    if sum(merge_titles.contains(titles(file_ind,i))) %here i is the variable we show 
        plot(shiftdim(results(file_ind,2*titleInColumn(1),1:nb_lines(file_ind)),1),shiftdim(results(file_ind,3*titleInColumn(1),1:nb_lines(file_ind)),1),'color',colors(file_ind,:));
    end
    hold on
end
xlabel(merge_titles(2));
ylabel(merge_titles(3));
%{
hL = subplot(plotSize(1),plotSize(2),L+1); % set room for legend
posLegend = get(hL,'position');
axis(hL,'off');                 % Turning its axis off
lgd = legend(hL,files_to_compare);
set(lgd,'position',posLegend);
%}
hold off
%% Other fewer display to get them bigger
figure
subplot(1,3,1)
for file_ind=1:nb_file
    if sum(merge_titles.contains(titles(file_ind,i))) %here i is the variable we show 
        plot(shiftdim(results(file_ind,2*titleInColumn(1),1:nb_lines(file_ind)),1),shiftdim(results(file_ind,3*titleInColumn(1),1:nb_lines(file_ind)),1),'color',colors(file_ind,:));
    end
    hold on
end
hold off
xlabel(merge_titles(2));
ylabel(merge_titles(3));
for i = 2*titleInColumn(1):titleInColumn(1):3*titleInColumn(1)
    subplot(1,3,i/titleInColumn(1));   %,'replace')
    for file_ind=1:nb_file
        if sum(merge_titles.contains(titles(file_ind,i))) %here i is the variable we show 
            plot(shiftdim(results(file_ind,titleInColumn(1),1:nb_lines(file_ind)),1),shiftdim(results(file_ind,i,1:nb_lines(file_ind)),1),'color',colors(file_ind,:));
        end
        hold on
    end
    ylabel(merge_titles(i/titleInColumn(1)));
    xlabel(merge_titles(1));
    hold off
end
legend(files_to_compare);

measurementx = reshape(results(2,1:2,:),2,[]);
measurementy = reshape(results(2,[1 3],:),2,[]);
referencex = reshape(results(1,1:2,:),2,[]);
referencey = reshape(results(1,[1 3],:),2,[]);

try 
    fprintf("Error projected along the X axis:\n");
    time_and_distance_error_f(referencex,measurementx);
    fprintf("\nError projected along the Y axis:\n");
    time_and_distance_error_f(referencey,measurementy);
catch
    fprintf("No function time_and_distance_error_f ... further analysis can't be done automaticaly\n");
end


%TODO: check what's the hell with titleInColumn, it should not matter
%anymore in the result matrix, this matrix should only have values.
