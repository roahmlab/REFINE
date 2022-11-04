function FRS = linear_regime_verification(FRS, manu_type)
% this script test if the FRS results in linear regime, otherwise modify
% the FRS to make it super large so that it is useless for online planning
% (equivalently, the corresponding FRS serie is deleted)
    load_const;
    Z = FRS.Z;
    dim = size(Z,1);
    
    FRS_itv = interval(FRS);
    h = FRS_itv(3);
    u = FRS_itv(4);
    v = FRS_itv(5);
    r = FRS_itv(6);
    t0 = Z(10,1);
    tbrk1 = Z(13,1);
    t = FRS_itv(end);
    u0 = FRS_itv(7);
    h0 = Z(end-1);
    pu = FRS_itv(11);
    py = FRS_itv(12);
    err_r_sum = FRS_itv(end-4);
    err_u_sum = FRS_itv(end-2);



    % trim u for stabalization if u appears in denomenator
    uub = supremum(u);
    ulb = infimum(u);
    if uub <= 1
        uden = 1;
    elseif ulb < 1
        uden = interval(1,uub);
    else
        uden = u;
    end

    % test rear slip angle
    alpha_r = -(v - lr*r) / uden;
    if supremum(alpha_r) < 0.1
        FRS = zonotope([FRS, [1000; 1000; zeros(dim-2,1)]]);
        return
    end
    Fyr = Car1 * alpha_r;


    [hd, ud, dud, rd, drd] = get_ref(t0,tbrk1,t,u0,h0,pu,py,manu_type);
    % test front slip angle
    Mr = ???
    tau_r = ((kappaP+kappaI*err_r_sum)*Mr + (phiP+phiI*err_r_sum)) * (r-rd);
    alpha_f = -Izz*Kr/lf/Caf1*(r-rd) - Izz*Kh/lf/Caf1*(h-hd) +  Izz/lf/Caf1*rd + lr/lf/Caf1*Fyr + Izz/lf/Caf1*tau_r;
    if supremum(alpha_f) < 0.1
        FRS = zonotope([FRS, [1000; 1000; zeros(dim-2,1)]]);
        return
    end

    % test front slip ratio (todo: define mu_bar=10 in load_const)
    Mu = ???
    tau_u = ((kappaPU+kappaIU*err_u_sum)*Mu + (phiPU+phiIU*err_u_sum)) * (u-ud);
    lambda_f = (lf+lr)/grav_const/lr/mu_bar*(-Ku*(u-ud) + dud -v*r + tau_u);
    if supremum(lambda_f) < 0.15
        FRS = zonotope([FRS, [1000; 1000; zeros(dim-2,1)]]);
        return
    end




end