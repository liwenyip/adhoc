P1 = [0 0.40 0.70 1.10 1.35 1.70 2.00 2.35 2.65 2.95 3.30];
P2 = [0 3.30 4.70 6.20 7.55 9.00 10.50 11.95 13.45 15.05 29.80 ];
r1 = [0 1 2 3 4 5 6 7 8 9 10];
r2 = [0 10 15 20 25 30 35 40 45 50 100];

subplot(1, 2, 1);
plot(r1, P1, 'sk-');
title('Power Consumption vs Duty Cycle (0%-10%)');
xlabel('Duty Cycle (%)');

ylabel('Power Consumption (mW)');

%pause

subplot(1, 2, 2);
plot(r2, P2, 'sk-');
title('Power Consumption vs Duty Cycle (0% - 100%)');
xlabel('Duty Cycle (%)');
ylabel('Power Consumption (mW)');

