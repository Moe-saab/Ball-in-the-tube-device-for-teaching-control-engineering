%% Run Just once

clear all; close all; clc;

a = arduino('/dev/ttyACM0', 'Uno'); %serial connection creation

generator = 'A4'; % generator positive wire to pin A4 of arduino
configurePin(a, generator, 'AnalogInput');

BlowerFan = 'D9'; % connect pwm of mosfet driver to pin 9 of 
                        %arduino that will outputs the pwm signal later in code

%%
gain = 1.1; % change gain and pole or even tf dependings on your identified model
pole = 0.5  ;
MotorMosfet = tf( gain , [ pole ,1 ] );

fs = 50; % sampling frequency (use same as one you used in identification)
Ts = 1/fs;

% Design Proportional controller 
Kp = 2.2;
MotorMosfet.OutputDelay = Ts;
margin(Kp*MotorMosfet);


% Discretize the model
model = c2d(MotorMosfet,Ts,'tustin'); 
ExpPeriod = 50;
N = ExpPeriod * fs;
time = [0:N] * Ts ;


%% Run simulink model before completing so the following are done and
% imported to the workspace:
% 1) Reference Speed signal desgined in simulink
% 2) Controlled Model Speed signal
%%

writePWMDutyCycle(a,BlowerFan,0.3); %operating point
pause(5);

refspeed = out.signal; % imported from simulink after being ran
modeloutcontrolled = out.modelControlledOut;

generatorVoltage = zeros(size(refspeed)); %measured speed
sErr = zeros(size(generatorVoltage)); % speed error
t = zeros(size(sErr));
u = zeros(size(sErr)); % input signal being the duty cycle fed to the mosfet from the arduino

tic;
for i = 1:length(refspeed)
    generatorVoltage(i) =  readVoltage(a,generator);
    sErr(i) = refspeed(i) - generatorVoltage(i);
    u(i) = Kp*sErr(i);
    if u(i) > 0.5 %saturation is 0.8 - operating point = 0.8 - 0.3 = 0.5
        u(i) = 0.5;
    elseif u(i) <0
            u(i) = 0;
    end
    writePWMDutyCycle(a,BlowerFan,u(i)+0.3);
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
figure(1);
plot(time,refspeed,time,generatorVoltage,time,modeloutcontrolled,"LineWidth",2);
title("Controlled System")
legend("reference Speed","Measured Speed Real","Measured Speed Model");

figure(2);
u_model = out.modelControlledOut1;
plot(time,u,time,u_model,"LineWidth",2)
title("Controller Output");
legend("real ", "model")




%% Using PI controller instead of P only
den = MotorMosfet.Denominator;
B = den{1}; B = B(1);

Ti = B;
Kp = 2.2;
PI = Kp * tf([Ti 1],[Ti 0]);

openloop = PI*MotorMosfet;
%rlocus(openloop);

Ts = 0.02;
openloop.OutputDelay = (Ts);
margin(openloop);
%%
PI_disc = c2d(PI,Ts,'tustin'); %discrete pi controller
num = PI_disc.Numerator{1};
den = PI_disc.Denominator{1};

aa = num(1); b= num(2); c = den(2);
%%

writePWMDutyCycle(a,BlowerFan,0.3);
pause(5);

refspeed = out.signal;
generatorVoltage = zeros(size(refspeed));
sErr = zeros(size(generatorVoltage));
t = zeros(size(sErr));
u = zeros(size(sErr));

i = 1;
generatorVoltage(i) =  readVoltage(a,generator);
sErr(i) = refspeed(i) - generatorVoltage(i);
u(i) = Kp*sErr(i);
if u(i) > 0.5 
        u(i) = 0.5;
    elseif u(i) <0
            u(i) = 0;
end
writePWMDutyCycle(a,BlowerFan,u(i)+0.3);

tic;
for i = 2:length(refspeed)
    generatorVoltage(i) =  readVoltage(a,generator);
    sErr(i) = refspeed(i) - generatorVoltage(i);
    %u(i) = kp*sErr(i);
    u(i) = -c*u(i-1) + aa*sErr(i) + b*sErr(i-1);
    if u(i) > 0.5 
        u(i) = 0.5;
    elseif u(i) <0
            u(i) = 0;
    end
    writePWMDutyCycle(a,BlowerFan,u(i)+0.3);
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
modeloutcontrolled = out.modelControlledOut;
time = 0:Ts:50;
%subplot(211);
figure(1);
plot(time,refspeed,time,generatorVoltage,time,modeloutcontrolled,"LineWidth",2);
title("Controlled System")
legend("reference Speed","Measured Speed Real","Measured Speed Model");
xlabel("seconds");

figure(2);
u_model = out.modelControlledOut1;
%subplot(212);
plot(time,u,time,u_model,"LineWidth",2)
title("Controller Output");
legend("real ", "model")

xlabel("seconds");
















