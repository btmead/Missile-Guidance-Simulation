function [results] = results_table(log)
%results_table: Creates a results table
%   Take fieldnames from log struct and creates a results table ready for
%   saving

names = string(fieldnames(log));
l = length(names);

results = table;

for i = 1:l
    column = log.(names(i));
    column = column(:);
    results.(names(i)) = column;
end