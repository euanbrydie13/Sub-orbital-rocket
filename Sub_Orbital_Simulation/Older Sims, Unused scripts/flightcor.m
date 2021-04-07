%% APPENDIX B TO PART 420—METHOD FOR DEFINING A FLIGHT CORRIDOR
% Author: Ali Rajabi - 06/02/2021

%% Mean geometric height (EQUATION B1)

% User input for number of months
k = 12;

% h = [] % You want to specify an [.. x k] array here
% n = [] % You want to specify an [.. x k] array here

% For test case going to make a random 1x10 array for both h and n
h=rand([1 12]);
n=rand([1 12]);

% Initialise
numerator = 0;
denominator = 0;

for i=1:1:k % Start from 1 month and finish at k, in increments of 1
    
    h_current = h(i); % Extract the data at the current month from array
    n_current = n(i); % Extract the data at the current month from array
    
    % Update numerator and denominator at current iteration
    numerator = numerator + (h_current*n_current);
    denominator = denominator + n_current;
    
end

% Having finished calculations / summation in for loop
% Calculate H
H_finished =  numerator/denominator;


%% 
