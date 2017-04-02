Freqs = [0 2.27 2.31 2.75 2.79 4.52 4.57 6];
Clock = [0 0 1 1 0 0 1 1];

plot(Freqs, Clock, '-bx');
axis([0 6 0 1.2]);
xlabel('Frequency (kHz)');
ylabel('Clock Detect Flag');
title('Clock Detect vs Frequency');
set(gca,'ytick',[0, 1])