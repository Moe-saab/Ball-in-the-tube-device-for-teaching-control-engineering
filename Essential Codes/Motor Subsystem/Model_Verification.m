%% Run Just once

clear all; close all; clc;

a = arduino('/dev/ttyACM0', 'Uno'); %serial connection creation

generator = 'A4'; % generator positive wire to pin A4 of arduino
configurePin(a, generator, 'AnalogInput');

BlowerFan = 'D9'; % connect pwm of mosfet driver to pin 9 of 
                        %arduino that will outputs the pwm signal later in code

                        
%% Model validation  
gain = 1.1; % change gain and pole or even tf dependings on your identified model
pole = 0.5  ;
MotorMosfet = tf( gain , [ pole ,1 ] );

fs = 50; % sampling frequency (use same as one you used in identification)
Ts = 1/fs;

model = c2d(MotorMosfet,Ts,'tustin'); %simulated discrete model

ExpPeriod = 40;
N = ExpPeriod * fs;
time = [0:N-1] * Ts ;

% Design of input signal being the duty cycle to send. You can change it
% but make sure it varies well and covers all your linear region you set so
% you verify well your model against real system (Don't just use simple
% step input)

u = ones(size(time));
t = floor(length(time)/9);
magnitudes = [0.4; 0.5; 0.6; 0.7; 0.8; 0.7; 0.6; 0.5; 0.4] ;

for i = 1:length(magnitudes)
    u((i-1)*t+1:i*t) = magnitudes(i);    
end

y = lsim(model,u-0.3,time)'; % Model response
real = zeros(1,N); % real system response 

writePWMDutyCycle(a,BlowerFan,0.3); % Always let your system startup from the operating point
pause(5);

tic;
for i = 1:N
    writePWMDutyCycle(a,BlowerFan,u(i));
    real(i) =  readVoltage(a,generator);
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


plot(time,real,time,y+real(1),"LineWidth",2);
legend("real","model");
title("Model Verification Motor");
xlabel("time sec");
ylabel("Voltage");







%% Different Duty Cycle input test for model validation as well
u1 = 0.35*ones(1,200);
u2 = 0.7*ones(1,300);
u3 = 0.5*ones(1,200);
u4 = 0.6*ones(1,400);
u5 = 0.4*ones(1,200);
u6 = 0.9*ones(1,150);
u7 = 0.7*ones(1,150);
u8 = 0.6*ones(1,200);
u9 = 0.5*ones(1,200);

uu = [u1 u2 u3 u4 u5 u6 u7 u8 u9];
 

real = zeros(1,N);
writePWMDutyCycle(a,BlowerFan,0.3);
pause(5);

tic;
for i = 1:N
    writePWMDutyCycle(a,BlowerFan,uu(i));
    real(i) =  readVoltage(a,generator);
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

y = lsim(model,uu-0.3,time)';
plot(time,real,time,y+real(1),"LineWidth",2);
legend("real","model");
title("Model Verification Motor");
xlabel("time sec");
ylabel("Voltage");
