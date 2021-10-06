function B = sparseBasis(A)
if(size(A,1) < size(A,2))
    error('ERROR: Wrong input for sparseBasis(). Use tall matrix')
end
thld = 0.000001;
[m, n] = size(A);
idx_count = 0;
rank_A = rank(A);
new_basis = zeros(m,1);
for i = n-1 : m-1
    idxpool = nchoosek(1:m, i);
    for j = 1:nchoosek(m, i)
        idx = idxpool(i,:);
        y = null(A(idx,:));
        if size(y,2) ~= n && size(y,2) ~= 0
            for k = 1:size(y,2)
                idx_count = idx_count + 1;
                new_basis(:,idx_count) = A * y(:,k);
                zero_count(idx_count) = sum(new_basis(:,idx_count) < thld);
            end
        end
    end
    rank(new_basis)
    if rank(new_basis) == rank(A)
        break;
    end
end
ordered_basis = sort([zero_count; new_basis], 2, 'descend');
ordered_basis = ordered_basis(2:end,:);
B(:, 1) = ordered_basis(:,1);
for i = 2:size(ordered_basis,2)
    prev_rank = rank(B);
    temp_B = [B ordered_basis(:,i)];
    curr_rank = rank(temp_B);
    if prev_rank < curr_rank
        B = temp_B;
    end
    if curr_rank == rank_A
        break;
    end
end
if(rank(B) ~= rank(A))
    error('ERROR: Failed to find independent bases')
end
end
