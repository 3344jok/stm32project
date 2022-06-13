function  data = sinesweep(f0, f1, sweeptime, samplingrate, peak)

    k = exp(log(f1/f0)/sweeptime)

    dt = 1.0/samplingrate
    t = 0.0
    p = 2*pi*f0/log(k)
    for i = 1 : sweeptime*samplingrate
         data(i) = peak*sin( p*(k^t-1) );
         t = t + dt;
    end
end
