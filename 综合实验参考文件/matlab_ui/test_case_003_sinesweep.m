clear all
clc
f0 = 0.1;
f1 = 10;
sweeptime = 10;
samplingrate = 100;
peak = 10;

data = sinesweep(f0, f1, sweeptime, samplingrate, peak);

plot(data);

