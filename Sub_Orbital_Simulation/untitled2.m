
Bank = [-0.2,-0.2,0,0.18,0,0,0,0,0,0,0,0,-0.2,-0.2,0];  
Throttle = [1,1,1,0,0,0,0,0,0,0,0,0,0,0,0];
Alpha = [0,0,0,-0.04,-0.04,-0.08,0,0.05,0,0,0,0,0,0,0];

TspanC = linspace(0,450,15);


figure('Name','Control Settings','NumberTitle','off')
hold on 
pAlpha = plot(TspanC,Alpha);
pThrottle = plot(TspanC,Throttle);
pBank = plot(TspanC,Bank);
pAlpha.DisplayName = "Angle of Attack";
pThrottle.DisplayName = "Throttle";
pBank.DisplayName = "Bank Angle";
legend