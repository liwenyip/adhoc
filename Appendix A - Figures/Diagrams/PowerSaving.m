% Plot variables:
f1 = 1/1800 ;           % Packet rate (Packets per second)
f2 = 1/60;
f3 = 1/1;

na = 5;              % Network density (Average Degree)  
nb = 20;
nc = 50;
tmax = 100;




V = 5;              % Supply voltage
Irx = 0.007;        % Receiver supply current = 7 mA
Itx = 0.008;        % Transmitter supply current = 8 mA
Ivga = 0.035;       % VGA supply current = 35 mA
T = 0.010;          % Listen time = 1.05 ms
t = 10.^[-3:0.1:2];   % Time vector
%t = [0:0.001:1000];   % Time vector
Pr = Irx * V;           % Receive Power
Pt = (Itx + Ivga) * V;   % Transmit Power

P1a = t./(t+T).*Pr - f1.*t.*(Pt+na*Pr/2);
P2a = t./(t+T).*Pr - f2.*t.*(Pt+na*Pr/2);
P3a = t./(t+T).*Pr - f3.*t.*(Pt+na*Pr/2);
P1b = t./(t+T).*Pr - f1.*t.*(Pt+nb*Pr/2);
P2b = t./(t+T).*Pr - f2.*t.*(Pt+nb*Pr/2);
P3b = t./(t+T).*Pr - f3.*t.*(Pt+nb*Pr/2);
P1c = t./(t+T).*Pr - f1.*t.*(Pt+nc*Pr/2);
P2c = t./(t+T).*Pr - f2.*t.*(Pt+nc*Pr/2);
P3c = t./(t+T).*Pr - f3.*t.*(Pt+nc*Pr/2);


set(gca,'LineStyleOrder',{'-*',':','o'})

semilogx(t, P1a, 'k-.', t, P2a, 'k--', t, P3a, 'k-', t, P1b, 'k-.', t, P2b, 'k--', t, P3b, 'k-', t, P1c, 'k-.', t, P2c, 'k--', t, P3c, 'k-');
xlabel('Sleep Time (seconds)');
ylabel('Power Saving (W)');
ylims = ylim;
ylim([0 max(P1a)*1.1]);
grid off;
legend('Low Traffic', 'Medium Traffic', 'High traffic');
title('Power Saving vs Sleep Interval');

%t1 = -T + sqrt(2*T*Pr/f/(Pt-n*Pr/2));

t1 = -T + sqrt(2*T*Pr/( 1/1 )/(Pt-( 3 )*Pr/2))
