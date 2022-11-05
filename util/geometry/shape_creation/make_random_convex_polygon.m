function P = make_random_convex_polygon(n,c)
% P = make_random_convex_polygon(n,c)
%
% Make a random 2-D convex polygon with at most n sides, centered (roughly)
% at the point c \in \R^2
%
% Usage example:
% P = make_random_convex_polygon(5)
% figure(1) ; clf ; grid on ; axis equal ;
% plot(P(1,:),P(2,:),'b')
%
% Authors: Shreyas Kousik
% Created: 8 Dec 2020
% Updated: nah

    if nargin < 2
        c = zeros(2,1) ;
    elseif nargin < 1
        n = 3 ;
    end
    
    P = make_random_polygon(n,c(:)) ;
    K = convhull(P') ;
    P = P(:,K) ;
end