data = load('data/sim_data.mat');
original = data.Pos(1,:);
constructed = load('cheks.log');
num_errors = 0;

er = 2*10^-3;

for i = 1 : length(original)
    if (abs(constructed(i) - original(i)) > er )
        num_errors = num_errors + 1;
    end
end

if num_errors == 0
    
    fprintf('Passed')

else
    
    fprintf('Failed')
end