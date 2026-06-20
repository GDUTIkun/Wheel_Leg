function ref = generateref(x, v, vd)
    ref = zeros(2,1);
    step = 10;
    dt = 0.01;
    vl = linspace(v, vd, step);
    ref(2) = vl(3);
    ref(1) = x + ref(2)*dt;
end