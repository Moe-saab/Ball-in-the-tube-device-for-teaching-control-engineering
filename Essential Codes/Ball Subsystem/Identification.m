%% Run Just once
clear all; close all ;clc;

% Connect to arduino and set sensors and actuators pins
% For other options check help arduino
a = arduino('/dev/ttyACM0', 'Uno'); %serial connection creation

sharp = 'A1';
configurePin(a, sharp, 'AnalogInput');

generator = 'A4'; % generator positive wire to pin A4 of arduino
configurePin(a, generator, 'AnalogInput');

BlowerFan = 'D9'; % connect pwm of mosfet driver to pin 9 of 
                        %arduino that will outputs the pwm signal later in code


%%
fs = 20; % sampling frequency
Ts = 1/fs;
exp_t = 8; %time per experiment
N = exp_t*fs; % 8 seconds per experiment
t = zeros(1,N);
time = [0:N-1] * Ts ;

speedAll = [0.7 0.8 0.9 1 1.1]; %reference speeds (act as input to this subsystem) and output is measured distance
% to know what values to choose for speedAll check speeds reached for duty
% cycle inputs in linear region in previous tests in motor subsystem.

sharpVoltage = zeros(length(speedAll),N); % distance measurement sensor

inputOffset = 0.3;

u = zeros(size(time)); %duty cycle
sErr = zeros(size(time)); % speed error

writePWMDutyCycle(a,BlowerFan,inputOffset);
pause(10);


kp = 2;

%%
for k = 1:length(speedAll)
    speed = speedAll(k);

    tic;
    for i = 1:N
        sErr(i) = speed - readVoltage(a,generator);
        u(i) = kp*sErr(i);
        if u(i) > 0.5 
            u(i) = 0.5;
        elseif u(i) <0
                u(i) = 0;
        end
        writePWMDutyCycle(a,BlowerFan,u(i)+0.3);
        sharpVoltage(k,i) =  readVoltage(a,sharp);
        %
        % can linearize here distance readings, i forgot and noticed system still worked well
        % so up to you. Just use same linearization equation used in 
        % file identification of system as a whole folder
        %
        t(i) = toc;
        if  t(i)> i*Ts
            disp("Sampling time too small");
        else
            while toc <= i*Ts
                % Do nothing till end of current sampling period
            end
        end

    end

writePWMDutyCycle(a,BlowerFan,inputOffset);

fprintf("Finished with DutyCycle : %.2f \n", speedAll(k));

pause(10);  
end
writePWMDutyCycle(a,BlowerFan,0);


%%
plot(time,sharpVoltage,'.',"LineWidth",2);  
legend("0.7", "0.8", "0.9", "1","1.1");
hold on;
title("Step Response Motor Speed to Ball Position");
xlabel("time sec");
ylabel("distance volt");

 %% Butterworth filter , Instead of using the noisy data for identification
 % I filter it first, this is not needed in case you use analog filter in
 % lab
 % I do filtering for first subpart in time with higher filtering cutoff
 % freq since I don't want to filter too much since still in transient phase
 % and for 2nd part I filter too much since steady state is reached
order = 4;             
cutoffFreq = 0.11; 
[b, aa] = butter(order, cutoffFreq);

for i = 1:length(speedAll)
filteredSignal1(i,:) = filtfilt(b, aa, sharpVoltage(i,1:60));
end

order = 4;     
cutoffFreq = 0.03;      
[b, aa] = butter(order, cutoffFreq);

for i = 1:length(speedAll)
filteredSignal2(i,:) = filtfilt(b, aa, sharpVoltage(i,45:end));
end

filteredSignal = [filteredSignal1(:,1:end-16) filteredSignal2];
filteredSignal(2,:) = filteredSignal(2,:) - 0.03;
hold on;
plot(time,filteredSignal,"LineWidth",2);
legend("0.7", "0.8", "0.9", "1");



%% Find the transfer function
figure(2);
for k = 1:size(speedAll,2)
    
    speed = speedAll(k)-inputOffset;
    y = filteredSignal(k,:);
    u = speed*ones(size(y));
    offset = filteredSignal(k,1); %Operating point
    SystemOrder=[1 2]; %Number of zeros and of poles (0 and 1), respectively.
    sysIdent = IdentifySystem(u,y-offset,SystemOrder,Ts)
    hold on;
    plot(time,y-offset,'.');
    hold on;
    lsim(sysIdent,u,time,'r');
    legend("Step","approximate Identified tf");

end

%  In my test, i got the following identified tf 
%       0.1512 s + 0.6963
%   --------------------------
%   0.09 s^2 + 0.28 s + 1




