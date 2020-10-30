%% The aim of this script is to compare data from two different files.
% The way it does it is by finding similar titles and then plot the similar
% variables on the same graphs
% This code is written from plot_data_from_file.m

clear();
files_to_compare = [
    "log_module_position.txt"
    "log_drone_pos_and_velocity.txt"
    ]';

nb_of_file = length(files_to_compare);
files = [];
first_lines = cell(nb_of_file,1);
dataFormat = strings(nb_of_file,1);

for i = 1:nb_of_file %we will have to repeat all operation for every file
    files(i) = fopen(files_to_compare(i));
    %first_lines(i) = fgets(files(i));
    first_lines(i) = textscan(fgets(files(i)),'%s'); %translate into an array of "words"/numbers
    cell_firstLine = first_lines(i);
    cell_firstLine = cell_firstLine{1};
    L = size(cell_firstLine);     % the number of columns
    L = L(1);                   % good type
    for j = 1:L
        dataFormat(i)=dataFormat(i)+"%s";
    end

    
    %% Analyse the text file to get its format 
    titleInColumn = ones(nb_of_file,1);  %variable title reprint each line
    for j=1:L
        if size(str2num(char(cell_firstLine{j})))==1
            titleInColumn(i)=2; %lines only get data (IE. the title is only print on the first line
        end
        
    end
end % here we have to end the for because we need to initialize new variables only once

titles = strings(nb_of_file,L/titleInColumn(i));    %The name of each variable    
for i=1:nb_of_file
    for j = 1:titleInColumn(i):L
        titles(i,j) = cell_firstLine{j};
    end
end



% TODO: check that if a latter file has more columns, all the titles are
% savede, and there is no error btw ^^



