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
exp_t = 40; %time per experiment
N = exp_t*fs; 
time = [0:N-1] * Ts ;

tube = 1.3*tf([0.1512, 0.6963] , [0.09 0.28 1]);

model = c2d(tube,Ts,'tustin');


s = ones(size(time));
t = floor(length(time)/4);
magnitudes = [0.7; 0.8; 0.9; 1] ;

for i = 1:length(magnitudes)
    s((i-1)*t+1:i*t) = magnitudes(i);    
end


%% Checking Real system behaviour for the input ref speed simulated up
sErr = zeros(size(time)); %speed error
u = zeros(size(time));
kp = 2;

real = zeros(1,N);

writePWMDutyCycle(a,BlowerFan,0.3);
pause(5);

tic;
for i = 1:N
    sErr(i) = s(i) - readVoltage(a,generator);
    u(i) = kp*sErr(i);
    if u(i) > 0.5 
            u(i) = 0.5;
        elseif u(i) <0
                u(i) = 0;
    end
    writePWMDutyCycle(a,BlowerFan,u(i)+0.3);
    real(i) =  readVoltage(a,sharp);
    t(i) = toc;
    if  t(i)> i*Ts
        disp("Sampling time too small");
    else
        while toc <= i*Ts
            % Do nothing till end of current sampling period
        end
    end

end

writePWMDutyCycle(a,BlowerFan,0);
%%
plot(time,real,"LineWidth",1);  

hold on;

y = lsim(model,s-0.2,time)'; 
plot(time,y+0.65,"LineWidth",2); % 0.65 is the speed measured for the operating point, i.e the 0.3 duty cycle offset which is always on
title("Model Verification Speed to Distance");
xlabel("time sec");
ylabel("Voltage");

