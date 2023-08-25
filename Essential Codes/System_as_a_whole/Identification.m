%% Run Just once

clear all; close all ;clc;

% Connect to arduino and set sensors and actuators pins
% For other options check help arduino
a = arduino('/dev/ttyACM0', 'Uno'); %serial connection creation

sharp = 'A1'; % sharp distance sensor positive wire to pin A1 of arduino
configurePin(a, sharp, 'AnalogInput');

BlowerFan = 'D9'; % connect pwm of mosfet driver to pin 9 of 
                        %arduino that will outputs the pwm signal later in code

%% Identification input : duty cycle , output : measured distance
% So here identifying system as a whole no cascade

% Function f(x) = p1*x^2 + p2*x + p3  can be used to linearize the static
% characteristic of the sharp small range distance sensor (Not very
% necessary)
p1 =     -0.8946 ;
p2 =       3.748  ;
p3 =      -1.715  ;

fs = 25; % sampling frequency
Ts = 1/fs;
ExpPeriod = 12;
N = ExpPeriod*fs; 
t = zeros(1,N);
time = [0:N-1] * Ts ;

DutyCycleAll = [0.3 0.4 0.5 0.6 0.7 0.8];

sharpVoltage = zeros(length(DutyCycleAll),N); % to save measured voltage from distance sensor
LinearizedMeasuredVoltage = sharpVoltage; % optional as I saw experimentally 
% that using identified transfer function at about 0.5 duty cycle input step response is still ok 

inputOffset = 0.2;

writePWMDutyCycle(a,BlowerFan,inputOffset);
pause(6);


for k = 1:length(DutyCycleAll)
    sharpVoltage(k,1) = readVoltage(a,sharp);
    LinearizedMeasuredVoltage(k,1) =  p1*sharpVoltage(k,1).^2 + p2*sharpVoltage(k,1) + p3;
    writePWMDutyCycle(a,BlowerFan,DutyCycleAll(k));
    tic;
    for i = 1:N-1
        sharpVoltage(k,i+1) =  readVoltage(a,sharp);
        if sharpVoltage(k,i+1) <2.2 
        LinearizedMeasuredVoltage(k,i+1) = p1*sharpVoltage(k,i+1).^2 + p2*sharpVoltage(k,i+1) + p3;
        else
            LinearizedMeasuredVoltage(k,i+1) = sharpVoltage(k,i+1);
        end
        
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

pause(6); 
end
writePWMDutyCycle(a,BlowerFan,0);


figure(1);
subplot(121);
plot(time,sharpVoltage,".","LineWidth",1);
 legend("0.3","0.4","0.5","0.6","0.7","0.8");
title("Step Response");
xlabel("time sec"); ylabel("Voltage in volt");
subplot(122);
plot(time,LinearizedMeasuredVoltage,".","LineWidth",1);
title("Step Response with Static Characteristic Linearized");
xlabel("time sec"); ylabel("Voltage in volt");
 legend("0.3","0.4","0.5","0.6","0.7","0.8");
 


%% Finding transfer functions
%TF chosen in my case = 2.5 / 0.09s2 + 0.6s + 1 

figure(3)
for k = 1:size(DutyCycleAll,2)
    
    DutyCycle = DutyCycleAll(k)-inputOffset;
    y = LinearizedMeasuredVoltage(k,:);
    u = DutyCycle*ones(size(y));
    offset = LinearizedMeasuredVoltage(k,1); %Operating point
    SystemOrder=[0 2]; %Number of zeros and of poles (0 and 1), respectively.
    sysIdent = IdentifySystem(u,y-offset,SystemOrder,Ts)
    hold on;
    plot(time,y-offset,'b.',"LineWidth",3);
    hold on;
    lsim(sysIdent,u,time,'r');
    legend("Step","approximate Identified tf");

end