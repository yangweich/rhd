%% MATLAB RHD IMPORT DATA
clear all;
close all;

disp('MATLAB Import RHD log')

FILENAME = 'rhdlog.txt';

T = readtable(FILENAME, 'Delimiter', ' ', 'ReadVariableNames', false, 'HeaderLines', 18);
rows = height(T);
vars = width(T);

%Construct Timevector t
t = table2array(T(:,1))';

offset = t(1);
for i = 1:rows
    t(i) = t(i)-offset;
end

% Sample rate
Fs = t(2);
% sample freqz
f = 1/Fs;

% Prep data for calc and plot

data = table2array(T(:,3))';

plot(t,data)


[h, w] = freqz(data);
%freqz(data);

% Design a butterworth filter
Wp = 20/(f);
Ws = 40/(f);
Rp = 1;
Rs = 60;
[n,Wn] = buttord(Wp,Ws,Rp,Rs) 
[b,a] = butter(n, Wn, 'low')
freqz(b,a)

y = filter(b,a,data);

%%
close all;
figure;
hold on;
plot(t,data,'blue')
plot(t,y,'red')
hold off;

%%
close all;
[hfilt,wfilt] = freqz(y);
 
hold on;
plot(w*(f*0.5)*(2/pi),20*log10(abs(h)),'blue');
plot(wfilt*(f*0.5)*(2/pi),20*log10(abs(hfilt)),'red');
legend('Raw','Filtered');
xlabel('Frequency (Hz)');
ylabel('Magnitude (dB)');


hold off;
    