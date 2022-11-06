function err_zono = get_error_zono(head_zono, gen, dim_total)
    headingc = center(head_zono);
    headingg = abs(generators(head_zono));
    if isempty(headingg)
        headingg = 0;
    end
    [len , width]=get_footprint_gen(gen,headingg);
    h =  headingc;
    ego_gen = [[cos(h)*len; sin(h)*len], [sin(-h)*width; cos(-h)*width]];
    gen = zeros(dim_total,2);gen(1:2,1:2) = ego_gen;
    err_zono  = zonotope([zeros(dim_total,1), gen]);
end
