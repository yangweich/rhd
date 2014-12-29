// MATLAB RHD IMPORT DATA

FILENAME = 'rhdlog.txt';

T = readtable(FILENAME, 'Delimiter', ' ', 'ReadVariableNames', false, 'HeaderLines', '18');

