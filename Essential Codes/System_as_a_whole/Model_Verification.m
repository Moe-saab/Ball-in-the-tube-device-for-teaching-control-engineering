%% Run Just once

clear all; close all ;clc;

% Connect to arduino and set sensors and actuators pins
% For other options check help arduino
a = arduino('/dev/ttyACM0', 'Uno'); %serial connection creation

sharp = 'A1'; % sharp distance sensor positive wire to pin A1 of arduino
configurePin(a, sharp, 'AnalogInput');

BlowerFan = 'D9'; % connect pwm of mosfet driver to pin 9 of 
                        %arduino that will outputs the pwm signal later in code

%% Model Verification of identified tf from duty cycle input to measured distance output
p1 =     -0.8946 ;
p2 =       3.748  ;
p3 =      -1.715  ;

fs = 25; % sampling frequency
Ts = 1/fs;
ExpPeriod = 40;
N = ExpPeriod*fs; 
t = zeros(1,N);
time = [0:N-1] * Ts ;

DutyCycleAll = [0.3 0.4 0.5 0.6 0.7 0.8];
 
% TF identified previously in "identification.m"
gain = 2.3;
aa = 0.09;
b = 0.6;
tube = tf( gain , [ aa, b ,1 ] );
model = c2d(tube,Ts,'tustin'); % discretized model


% Design input signal to verify against it (Don't just use simple step it
% is not enough to see behaviour for different operating points)
u = ones(size(time));
t = floor(length(time)/9);
magnitudes = [0.4; 0.5; 0.6; 0.7; 0.8; 0.7; 0.6; 0.5; 0.4] ;

for i = 1:length(magnitudes)
    u((i-1)*t+1:i*t) = magnitudes(i);    
end


real = zeros(1,N);
linreal = real; %linearized real distance "same equation use for 
% linearizing static characteristic of the sharp infrared sensor readings"

writePWMDutyCycle(a,BlowerFan,0.3);
pause(5);

tic;
for i = 1:N
    writePWMDutyCycle(a,BlowerFan,u(i));
    real(i) =  readVoltage(a,sharp);
    if real(i) <2.2
        linreal(i) = p1*real(i).^2 + p2*real(i) + p3;
        else
            linreal(i) = real(i);
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

writePWMDutyCycle(a,BlowerFan,0);


y = lsim(model,u-0.3,time)';
plot(time,linreal,time,y+linreal(1),"LineWidth",2);
legend("real","model");
title("Model Verification Motor");
xlabel("time sec");
ylabel("Voltage");












%% Different Input test for even better verification of the model (Not gradual steps of duty cycle)
u1 = 0.4*ones(1,200);
u2 = 0.7*ones(1,300);
u3 = 0.5*ones(1,200);
u4 = 0.6*ones(1,400);
u5 = 0.4*ones(1,200);
u6 = 0.8*ones(1,150);
u7 = 0.7*ones(1,150);
u8 = 0.6*ones(1,200);
u9 = 0.5*ones(1,200);

uu = [u1 u2 u3 u4 u5 u6 u7 u8 u9];
N = length(uu);
time = [0:N-1] * Ts ;

real = zeros(1,N);
linreal = real;
writePWMDutyCycle(a,BlowerFan,0.3);
pause(5);

tic;
for i = 1:N
    writePWMDutyCycle(a,BlowerFan,uu(i));
    real(i) =  readVoltage(a,sharp);
    if real(i) <2.2
        linreal(i) = p1*real(i).^2 + p2*real(i) + p3;
        else
            linreal(i) = real(i);
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

writePWMDutyCycle(a,BlowerFan,0);
 
y = lsim(model,uu-0.23,time)';
plot(time,linreal,time,y+linreal(1),"LineWidth",2);
legend("real","model");
title("Model Verification Motor");
xlabel("time sec");
ylabel("Voltage");
