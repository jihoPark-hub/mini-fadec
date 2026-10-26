parameters;
for N0 = [60 85 100]
    Wf0 = (N0/100)^n;
    [A,B,C,D] = linmod('spool_linmod', N0, Wf0);
    G = tf(ss(A,B,C,D))
end