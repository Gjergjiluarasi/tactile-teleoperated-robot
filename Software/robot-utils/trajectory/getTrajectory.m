function [b5, b4, b3, b2, b1, b0] = getTrajectory(x0,v0,a0,xf,vf,af,vMax, aMax, jMax)
%     tvMax = (xf-x0)/vMax;
    tvMax = (vMax-v0)/aMax;
    taMax = (aMax-a0)/jMax;
    if tvMax > taMax
        disp("tvMax");
        t = tvMax*5;
    else
        disp("taMax");
        t = taMax*10;
    end

    b = [xf-x0-v0*t-a0/2*t^2; vf-v0-a0*t; af-a0];
    T = [t^3 t^4 t^5; 3*t^2 4*t^3 5*t^4; 6*t 12*t^2 20*t^3];
    B = T\b;
    b0 = x0;
    b1 = v0;
    b2 = a0/2;
    b3 = B(1);
    b4 = B(2);
    b5 = B(3);

end