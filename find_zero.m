%% Swarm Formation Control 
% Description : hungarian algorithm
% Author      : Weifan Zhang 
% Date        : February 7, 2018
% Other Files :

function [ index,indep_zero_rows_index,indep_zero_cols_index ] = find_zero( cost,index,indep_zero_rows_index,indep_zero_cols_index )
    size = length(cost);
    while true
        %% count zeros in every row and col
        zero_num_rows = ones(size,1)*(size+1);
        zero_num_cols = ones(size,1)*(size+1);
        [axis_rows, axis_cols] = find(cost == 0);
        for i=1:size
            if indep_zero_rows_index(i)~=1 && ~isempty(find(axis_cols==i,1))
                zero_num_rows(i) = length(find(axis_cols == i));
            end
            if indep_zero_cols_index(i)~=1 && ~isempty(find(axis_rows==i,1))
                zero_num_cols(i) = length(find(axis_rows == i));
            end
        end
        
         %% find the row or col that has minimum zero number
        [min_row,min_row_index]=min(zero_num_rows);
        [min_col,min_col_index]=min(zero_num_cols);
        [min_t,row_or_col]=min([min_row,min_col]);
        %disp('~~0');
        if min_t > size
            break
        end
        %% find the independent zero element in that row os col
        if row_or_col==1 %is row
            indep_zero_rows_index(min_row_index)=1;
            for i=1:size
                if cost(min_row_index,i)==1
                    continue
                end
                if indep_zero_cols_index(i)==0
                    indep_zero_cols_index(i)=1;
                    index=[index,[min_row_index;i]];
                    % disp('~~1');
                    break
                end
            end           
        elseif row_or_col==2
            indep_zero_cols_index(min_col_index)=1;
            for i=1:size
                if cost(i,min_col_index)==1
                    continue
                end
                if indep_zero_rows_index(i)==0
                    indep_zero_rows_index(i)=1;
                    index=[index,[i;min_col_index]];
                    %disp('~~2');
                    break
                end
            end  
        end
    end
    
end

