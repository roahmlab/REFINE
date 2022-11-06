function FRS = linear_regime_verification(FRS, manu_type)
% this script test if the FRS results in linear regime, otherwise modify
% the FRS to make it super large so that it is useless for online planning
% (equivalently, the corresponding FRS serie is deleted)
    load my_const.mat;
    Z = FRS.Z;
    dim = size(Z,1);
    
    FRS_itv = interval(FRS);
    h = FRS_itv(3);
    u = FRS_itv(4);
    v = FRS_itv(5);
    r = FRS_itv(6);
    t0 = Z(10,1);
    tbrk1 = Z(13,1);
    t = FRS_itv(20);
    u0 = FRS_itv(7);
    h0 = Z(end-1);
    p_u = FRS_itv(11);
    p_y = FRS_itv(12);
    err_r_sum = FRS_itv(16);
    err_u_sum = FRS_itv(18);

    [hd, ud, dud, rd, drd] = get_ref(t0,tbrk1,t,u0,h0,p_u,p_y,manu_type);

    % trim u for stabalization when u appears in denomenator as suggested
    % by Tae-Yun Kim et al, Advanced slip ratio for ensuring numerical
    % stability of low-speed driving simulation
    uub = supremum(u);
    ulb = infimum(u);
    uden = interval(max(u_really_slow*1.1,ulb), max(u_really_slow*1.1,uub));

    if uub <= u_really_slow
        r = rd; % recall delta = rd * (lr+lf+Cus*u^2) / u and Cus = m/(lr+lf)*(lr/Caf1 - lf/Car1);
        v = lr*r - m*lf/Car1/(lf+lr)*u^2*r;
        alpha_r = -(v - lr*r) / uden;
    else
        vlb = infimum(v);
        vub = supremum(v);
        rlb = infimum(r);
        rub = supremum(r);

        if (vlb >=0 || vub <=0) && (rlb >=0 || rub <=0)
            alpha_r = -(v - lr*r) / uden;
        elseif vlb*vub < 0 && (rlb >=0 || rub <=0)
            if rlb >= 0
                alpha_r = -(interval(0,vub) - lr*r) / uden;
            else
                alpha_r = -(interval(vlb,0) - lr*r) / uden;
            end
        elseif (vlb >=0 || vub <=0) && rlb*rub<0
            if vlb >= 0
                alpha_r = -(v - lr*interval(0,rub)) / uden;
            else
                alpha_r = -(v - lr*interval(rlb,0)) / uden;
            end
        else
            alpha_r1 = -(interval(vlb,0) - lr*interval(rlb,0)) / uden;
            alpha_r2 = -(interval(0,vub) - lr*interval(0,rub)) / uden;
            alpha_r = interval(min(infimum(alpha_r1),infimum(alpha_r2)), max(supremum(alpha_r1),supremum(alpha_r2)));
        end
        
    end

    % test rear slip angle
    if supremum(alpha_r) > alpha_cri
        FRS = zonotope([FRS.Z, [1000; 1000; zeros(dim-2,1)]]);
        disp('alpha_r is too large')
        return
    end
    Fyr = Car1 * alpha_r;


    
    % test front slip angle
    tau_r = ((kappaP+kappaI*err_r_sum)*Mr + (phiP+phiI*err_r_sum)) * (r-rd);
    alpha_f = -Izz*Kr/lf/Caf1*(r-rd) - Izz*Kh/lf/Caf1*(h-hd) +  Izz/lf/Caf1*rd + lr/lf/Caf1*Fyr + Izz/lf/Caf1*tau_r;
    if supremum(alpha_f) > alpha_cri
        FRS = zonotope([FRS.Z, [1000; 1000; zeros(dim-2,1)]]);
        disp('alpha_f is too large')
        return
    end

    % test front slip ratio (todo: define mu_bar=10 in load_const)
    tau_u = ((kappaPU+kappaIU*err_u_sum)*Mu + (phiPU+phiIU*err_u_sum)) * (u-ud);
    lambda_f = (lf+lr)/grav_const/lr/mu_bar*(-Ku*(u-ud) + dud -v*r + tau_u);
    if supremum(lambda_f) > lambda_cri
        FRS = zonotope([FRS.Z, [1000; 1000; zeros(dim-2,1)]]);
        disp('lambda_f is too large')
        return
    end




end