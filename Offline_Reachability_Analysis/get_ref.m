function [hd, ud, dud, rd, drd] = get_ref(t0,tbrk1,t,u0,h0,pu,py,manu_type)
    load my_const.mat;
    hd = h0;
    ud = pu;
    dud = 0;
    rd = 0;
    drd = 0;
    t = t + t0;
    if isa(t, 'double')
        tlb = t;
        tub = t;
    elseif isa(t, 'interval')
        tlb = infimum(t);
        tub = supremum(t);
    else
        error('undefined variable type')
    end
    switch manu_type
        case 'Au' % speed change
            t1 = tpk_dir; % non-braking maneuver
            t2 = t1 + tbrk1; % first phase of braking maneuver (const deaccelerate)
            if tub <= t1 % non-braking
                ud = (pu - u0) / t1 * t + u0;
                dud = (pu - u0) / t1;                
            elseif tlb >= t2 % braking2
                ud = 0;
                dud = 0;
            else % braking `
                ud = pu - (pu-u_really_slow)*(t-t1)/tbrk1;
                dud = - (pu-u_really_slow)/tbrk1;
            end
        case 'dir' % dirction change
            t1 = tpk_dir;
            t2 = t1 + tbrk1;
            if tub <= t1
                rd = py/2 - (py*cos(2*pi*t/t1))/2;
                drd = (py*sin(2*pi*t/t1))* pi/t1;
                hd = h0 + py/2 - py*t1/4/pi*sin(2*pi*t/t1);
            elseif tlb >= t2
                ud = 0;
                hd = h0 + py/2*t1;
            else
                ud = pu - (pu-u_really_slow)*(t-t1)/tbrk1;
                dud = - (pu-u_really_slow)/tbrk1;
                hd = h0 + py/2*t1;
            end
        case 'lan' % lane change
            t1 = tpk;
            t2 = t1 + tbrk1;
            h1des = 6*sqrt(2*exp(1))/11;
            h2des = 121/144;
            if tub <= t1
                hd = h0 + h1des * py * exp(-h2des * (t-0.5*t1)^2);
                rd = h1des * py * (-2*h2des) * exp(-h2des * (t-0.5*t1)^2) * (t-0.5*t1);
                drd = h1des * py * (-2*h2des) * (exp(-h2des * (t-0.5*t1)^2) * (-2*h2des) *(t-0.5*t1)^2 ...
                      + exp(-h2des * (t-0.5*t1)^2));
            elseif tlb >= t2
                ud = 0;
            else
                ud = pu - (pu-u_really_slow)*(t-t1)/tbrk1;
                dud = - (pu-u_really_slow)/tbrk1;
            end
    end

end


