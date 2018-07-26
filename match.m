%% Swarm Formation Control 
% Description : hungarian algorithm
% Author      : Weifan Zhang 
% Date        : February 7, 2018
% Other Files :

function [ role_index ] = match( cost )
    %% step 1
    N = length(cost);
    %pre-operation
    rows = min((cost'));
    rows = repmat(rows',1, N);
    cost = cost - rows;
    cols = min(cost);
    cols = repmat(cols, N, 1);
    cost = cost - cols;

    indep_zero_rows_index = zeros(1,N);
    indep_zero_cols_index = zeros(1,N);
    index=[];
    while true
        %% step 2
        [ index,indep_zero_rows_index,indep_zero_cols_index ] = find_zero( cost,index,indep_zero_rows_index,indep_zero_cols_index );
        if size(index,2)==N
            break
        end
        
        %% step 3
         %check rows and cols so that they cover every zero element
        check_rows = ones(1,N);
        check_cols = zeros(1,N);
        for i=1:N
            if indep_zero_rows_index(i)==0
                check_rows(i)=0;
            end
        end
        flag=1;
        while flag==1
           flag=0;
           %check cols which has a normal zero element in unchecked rows
           for i=1:N
               if check_rows(i)==1
                   continue
               end
               for j=1:N
                   if cost(i,j)==0&&check_cols(j)==0
                       check_cols(j)=1;
                       flag=1;
                   end
               end
           end
           %uncheck rows which has an independent zero element in checked
           %cols
           for j=1:N
               if check_cols(j)==0
                   continue
               end
               for i=1:size(index,2)
                  if index(2,i)==j&&check_rows(index(1,i))==1
                      check_rows(index(1,i))=0;
                      flag=1;
                  end
               end                                
           end
        end
        %% step 4
        %find the minima from uncovered elements
        min_t=-1;
        for i=1:N
            if check_rows(i)==1
                continue
            end
            for j=1:N
                if check_cols(j)==1
                    continue
                end
                if cost(i,j)<min_t||min_t<0
                    min_t=cost(i,j);
                end
            end
        end
        %update cost
        for i=1:N
            if check_rows(i)==1
                for j=1:N
                    if check_cols(j)==1
                        cost(i,j)=cost(i,j)+min_t;
                    end
                end
            else
                for j=1:N
                    if check_rols(j)==0
                        cost(i,j)=cost(i,j)-min_t;
                    end
                end
            end
        end
        %clean
        indep_zero_rows_index = zeros(1,N);
        indep_zero_cols_index = zeros(1,N);
        index=[];
    end
    %% output
    role_index=zeros(1,N);
    for i=1:N
       role_index(index(1,i))=index(2,i); 
    end
end

