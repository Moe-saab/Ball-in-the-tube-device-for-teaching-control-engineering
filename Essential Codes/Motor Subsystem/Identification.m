%% Run Just once

clear all; close all ;clc;

% Connect to arduino and set sensors and actuators pins
% For other options check help arduino
a = arduino('/dev/ttyACM0', 'Uno'); %serial connection creation

generator = 'A4'; % generator positive wire to pin A4 of arduino
configurePin(a, generator, 'AnalogInput');

BlowerFan = 'D9'; % connect pwm of mosfet driver to pin 9 of 
                        %arduino that will outputs the pwm signal later in code


%% Checking linear region with step response for different duty cycles

fs = 50; % sampling frequency
Ts = 1/fs;
exp_t = 8; %time per experiment
N = exp_t*fs; % 8 seconds per experiment
t = zeros(1,N);
time = [0:N-1] * Ts ;

DutyCycleAll = [0.4, 0.45, 0.5, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8];
generatorVoltage = zeros(length(DutyCycleAll),N);

inputOffset = 0.3; % This value of duty cycle is the operating point
                                    % it is the value enough to rotatoe the
                                    % motor in enough speed to just
                                    % levitate the ball. (For 7.5 voltage
                                    % source)

writePWMDutyCycle(a,BlowerFan,inputOffset); % Send pwm signal of duty cycle = inputOffset
                                                                                          % to pin BlowerFan (pin D9 here) which will start 
                                                                                          % the motor.

 pause(6); %give time for voltage settlement before starting the tests


for k = 1:length(DutyCycleAll)
    writePWMDutyCycle(a,BlowerFan,DutyCycleAll(k));
    
    tic;
    for i = 1:N
        generatorVoltage(k,i) =  readVoltage(a,generator);
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

fprintf("Finished with DutyCycle : %.2f \n", DutyCycleAll(k));

pause(5);  
end
writePWMDutyCycle(a,BlowerFan,0);


figure(1);
plot(time,generatorVoltage,"LineWidth",2);
title("Step Response Blower Fan Variable Duty Cycle");
xlabel("time sec"); ylabel("Voltage in volt 'after voltage divider circuit'");
legend("0.4 Duty Cycle", "0.45", "0.5", "0.55", "0.6", "0.65", "0.7", "0.75", "0.8");






%% Identification 
figure(2);

for k = 1:size(DutyCycleAll,2)
  
    DutyCycle = DutyCycleAll(k)-inputOffset;
    y = generatorVoltage(k,:);
    u = DutyCycle*ones(size(y));
    offset = generatorVoltage(k,1); %Operating point
    SystemOrder=[0 1]; %Number of zeros and of poles (0 and 1), respectively.
    sysIdent = IdentifySystem(u,y-offset,SystemOrder,Ts)
    hold on;
    plot(time,y-offset,'.');
    hold on;
    lsim(sysIdent,u,time,'r');
    legend("Step","approximate Identified tf");

end



