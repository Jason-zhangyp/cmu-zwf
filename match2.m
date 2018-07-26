%% Swarm Formation Control 
% Description : hungarian algorithm
% Author      : Weifan Zhang 
% Date        : February 7, 2018
% Other Files :

function [ index ] = match2( cost )
    %% step 1
    size = length(cost);
    %pre-operation
    line = min((cost'));
    line = repmat(line',1, size);
    cost = cost - line;
    column = min(cost);
    column = repmat(column, size, 1);
    cost = cost - column;

    %%
    [axis_line, axis_column] = find(cost == 0);
    %C = [axis_line axis_column];
    result = [axis_line, axis_column];
    
    %decode the matrix to three dimension
    %test = cell(size, 1);
    z = zeros(size, size);
    for i=1:size
        x = find(result(:, 2) == i);
        temp_size = length(x);
        z(i, 1) = temp_size;
        for j = 2:temp_size+1
            test = result(x(j - 1),1);
            z(i, j) = test;
        end
    end


    %%
    %mark
    %flag = zeros(size, size);
    %this loop is aimed to finding the answer for sereral iteration
    result_t = result;
    while true
        %mark
        markedLine = zeros(1, size);
        markedColumn = zeros(1, size);
        [tempAnswer, all] = fit(z, 1, (1:size), zeros(1, size), []);
        left = setdiff((1:size), tempAnswer);
        if ~isempty(all)
            break;
        end
        %mark the row
        %flag(left, :) = 1;
        markedLine(left) = 1;
        %this loop is aimed to find the possible zero
        %find all marked lines
        tmp = find(markedLine == 1);
        while true
            %for i = 1:length(tempLine)
           %%
            %step1
            %find the line(subscript)
            tmp = find(result_t(:, 1) == tmp);
            erase = tmp;
            %find the column
            tmp = result_t(tmp, 2);
            %find weather it is terminate
            if isempty(tmp)
                break;
            end
            %mark the column
            %flag(:,tmp) = 1;
            markedColumn(tmp) = 1;
            %erase the line
            result_t(erase, :) = [];

           %%
            %step2
            %find the line(subscript)
            tmp = find(result_t(:, 2) == tmp);
            erase = tmp;
            %find the line
            tmp = result_t(tmp, 1);
            %find weather it is terminate
            if isempty(tmp)
                break;
            end
            %mark the line
            %flag(tmp, :) = 1;
            markedLine(tmp) = 1;
            %erase the column
            result_t(erase, :) = [];
        end
        %end

        %%
         %find the minimal number
         line = markedLine;
         column = not(markedColumn);
         flag = (line')*column;
         subscript = find(flag == 1);
         min_t = findMin(cost, subscript);
         %add and sub
         templine = find(markedLine == 1);
         tempcolumn = find(markedColumn == 1);
         cost(templine, :) = cost(templine, :) - min_t;
         cost(:, tempcolumn) = cost(:, tempcolumn) + min_t;
         temp = find(cost(subscript) == 0);
         temp = subscript(temp);
         %for i = 1:length(temp)
             y = fix((temp-1)/5) + 1;
             x = temp - (y-1)*5;
         %end
         result_t = [result; [x', y']];
         %check the answer                                             
         for i = 1:length(x)
             z(x(i), 1) = z(x(i), 1) + 1;
             z(x(i), z(x(i), 1)+1) = y(i); 
         end
    end
    result = all;
    index=result(1,:);

end

