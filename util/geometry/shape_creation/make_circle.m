function C = make_circle(r,n,p)
% C = make_circle(r,n,p)
%
% Make a circle of radius r, approximated as an n-sided polygon, centered
% at p \in \R^2. By default, r = 1, n = 100, and p = zeros(2,1). The output
% C is a 2-by-n array.
%
% Authors: Shreyas Kousik
% Created: who knows!
% Updated: 7 Jan 2021

    if nargin < 3
        p = zeros(2,1) ;
    end

    if nargin < 2
        n = 100 ;
    end
    
    if nargin < 1
        r = 1 ;
    end
    
    % make angle vector
    a_vec = linspace(0,2*pi,n) ;
    
    % make C
    C = [r*cos(a_vec) ; r*sin(a_vec) ] + repmat(p(:),1,n) ;
end