clear;
clc;
close all;

%Define physical parameters
parameters.mu = 0.001;
parameters.rho = 998;
parameters.g = 9.81;
parameters.r = 0.01;
parameters.d = 0.02;
mean = 0.01;
variance = 0.001;

%Define time interval
T = 0.1;
parameters.T = T;
timeend = 20;

%Define initial condition
h10 = 1;
h20 = 0;
h30 = 0;
h40 = 0;

%Define reference water levels, note must consider mean of the disturbance
a1 = 0.4+mean*timeend;
a2 = 0.3+mean*timeend;
a3 = 0.2+mean*timeend;
a4 = 0.1+mean*timeend;
a = [a1;a2;a3;a4];

%Define the eight matrices corresponding to each control
[A000,A001,A010,A011,A100,A101,A110,A111] = def_matrices(parameters);

%Simulation process, time interval [0,10]
h1 = zeros(1,timeend/T);
h2 = zeros(1,timeend/T);
h3 = zeros(1,timeend/T);
h4 = zeros(1,timeend/T);
d = zeros(1,8);

%Give Initial Condition
h1(1) = h10;
h2(1) = h20;
h3(1) = h30;
h4(1) = h40;

%Generate Disturbance,adjusted with respect to T
N = timeend/T; %Define the number of samples
mu = [mean*T mean*T mean*T mean*T];
Sigma = [variance*T 0 0 0;0 variance*T 0 0;0 0 variance*T 0;0 0 0 variance*T];
R = chol(Sigma);
array = repmat(mu,N,1) + randn(N,4)*R;

for i = 2:1:timeend/T
    %Calculate states for next stage
    %A000
    next000 = A000*[h1(i-1);h2(i-1);h3(i-1);h4(i-1)];
    %A001
    next001 = A001*[h1(i-1);h2(i-1);h3(i-1);h4(i-1)];
    %A010
    next010 = A010*[h1(i-1);h2(i-1);h3(i-1);h4(i-1)];
    %A011
    next011 = A011*[h1(i-1);h2(i-1);h3(i-1);h4(i-1)];
    %A100
    next100 = A100*[h1(i-1);h2(i-1);h3(i-1);h4(i-1)];
    %A101
    next101 = A101*[h1(i-1);h2(i-1);h3(i-1);h4(i-1)];
    %A110
    next110 = A110*[h1(i-1);h2(i-1);h3(i-1);h4(i-1)];
    %A111
    next111 = A111*[h1(i-1);h2(i-1);h3(i-1);h4(i-1)];
    %Compute minimum distance to reference and choose optimal control
    d(1) = norm(next000-a);
    d(2) = norm(next001-a);
    d(3) = norm(next010-a);
    d(4) = norm(next011-a);
    d(5) = norm(next100-a);
    d(6) = norm(next101-a);
    d(7) = norm(next110-a);
    d(8) = norm(next111-a);
    [M,controller] = min(d);
    %Set the next state
    switch controller
        case 1 
            [h1(i),h2(i),h3(i),h4(i)] = deal(next000(1),next000(2),next000(3),next000(4));
        case 2
            [h1(i),h2(i),h3(i),h4(i)] = deal(next001(1),next001(2),next001(3),next001(4));
        case 3
            [h1(i),h2(i),h3(i),h4(i)] = deal(next010(1),next010(2),next010(3),next010(4));
        case 4
            [h1(i),h2(i),h3(i),h4(i)] = deal(next011(1),next011(2),next011(3),next011(4));
        case 5
            [h1(i),h2(i),h3(i),h4(i)] = deal(next100(1),next100(2),next100(3),next100(4));
        case 6
            [h1(i),h2(i),h3(i),h4(i)] = deal(next101(1),next101(2),next101(3),next101(4));
        case 7
            [h1(i),h2(i),h3(i),h4(i)] = deal(next110(1),next110(2),next110(3),next110(4));
        case 8
            [h1(i),h2(i),h3(i),h4(i)] = deal(next111(1),next111(2),next111(3),next111(4));
    end
    h1(i) = h1(i) + array(i,1);
    h2(i) = h2(i) + array(i,2);
    h3(i) = h3(i) + array(i,3);
    h4(i) = h4(i) + array(i,4);
end

%Plot the states
timespan = linspace(0,timeend,timeend/T);
figure;
subplot(2,2,1);
plot(timespan,h1);
title('Tank1');
xlabel('s');
ylabel('Water Level in m');
subplot(2,2,2);
plot(timespan,h2);
title('Tank2');
xlabel('s');
ylabel('Water Level in m');
subplot(2,2,3);
plot(timespan,h3);
title('Tank3');
xlabel('s');
ylabel('Water Level in m');
subplot(2,2,4);
plot(timespan,h4);
title('Tank4');
xlabel('s');
ylabel('Water Level in m');
suptitle('Four Water Tank Cases: Stochastic Time Evolution');