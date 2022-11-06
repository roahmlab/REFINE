%% Ram's code an effort to find a good v, r
%% solving the vd(t) differential equation
%UNDER Construction
syms vd(t) Kv ev0 ud(t) Ku eu0 m lf lr Izz Ca r(t) r0 u(t) v(t) 
% Gaussian lane change maneuver (described in terms of the heading)
p(t) = r0 * exp( -( t - 1.5 )^2/ (1/3) );
r(t) = diff( p, t );

% vehicle parameters
load_const
% Ca = 6510;



v(t) = vd(t) + exp( -Kv * t ) * ev0;
u(t) = 10 + exp( -Ku * t ) * eu0;

diffeq = 2/Izz * ( ( lf + lr ) * Ca * ( ( v(t) - lr * r( t ) )/u( t ) ) + ...
lf * m * u( t ) * r( t )/2 - m/2 * Kv * ( v(t) - vd(t) ) + m/2 * diff( vd(t), t ));

vdt_solved = dsolve( diff( r, t ) == diffeq, vd( 0 ) == 0 );

vdt_solved = simplify( vdt_solved );

%% transforming the solution into something that is faster to parse in MATLAB
% integrand is vdt_solved from above but we've removed the call to the
% matlab function integral from the function...this allows us to make the
% subsequent computation faster 
syms x
% integrand = exp(- 3*x^2 - 10*x - 27/4)*(eu0 + 10*exp(10*x))^(4812434909657555/1125899906842624)*(177720080193383027428995457089536*r0*exp(29*x) - 12881076943450452168880536158208*ev0*exp(3*x^2 + 10*x + 27/4) + 301361028313534760670755553280*eu0*ev0*exp(3*x^2 + 27/4) - 6301911510318218776332635996160*eu0*r0*exp(19*x) - 131112457850638115971603061800960*r0*x*exp(29*x) - 396770603889075285859781050368*eu0^2*r0*exp(9*x) + 7840802723823777316142328053760*r0*x^2*exp(29*x) + 264513735926050190573187366912*eu0^2*r0*x*exp(9*x) + 784080272382377731614232805376*eu0*r0*x^2*exp(19*x) + 2938033901373870616621048922112*eu0*r0*x*exp(19*x))/(30136102831353476067075555328*(eu0 + 10*exp(10*t))^(5938334816500179/1125899906842624))
integrand = exp(- 3*x^2 - 10*x - 27/4)*(eu0 + 10*exp(10*x))^(5720347281834771/1338311809433600)*(177720080193383041662371798424465*r0*exp(29*x) - 12881076943450452168880536158208*ev0*exp(3*x^2 + 10*x + 27/4) + 301361028313534760670755553280*eu0*ev0*exp(3*x^2 + 27/4) - 6301911510318218776332635996160*eu0*r0*exp(19*x) - 131112457850638113450921616369590*r0*x*exp(29*x) - 396770603889075285859781050368*eu0^2*r0*exp(9*x) + 7840802723823777316142328053760*r0*x^2*exp(29*x) + 264513735926050190573187366912*eu0^2*r0*x*exp(9*x) + 784080272382377731614232805376*eu0*r0*x^2*exp(19*x) + 2938033901373870616621048922112*eu0*r0*x*exp(19*x))/(30136102831353476067075555328*(eu0 + 10*exp(10*t))^(7058659091268371/1338311809433600));
%exp(- 3*x^2 - 10*x - 27/4)*(eu0 + 10*exp(10*x))^(5720347281834771/1338311809433600)*(177720080193383041662371798424465*r0*exp(29*x) - 12881076943450452168880536158208*ev0*exp(3*x^2 + 10*x + 27/4) + 301361028313534760670755553280*eu0*ev0*exp(3*x^2 + 27/4) - 6301911510318218776332635996160*eu0*r0*exp(19*x) - 131112457850638113450921616369590*r0*x*exp(29*x) - 396770603889075285859781050368*eu0^2*r0*exp(9*x) + 7840802723823777316142328053760*r0*x^2*exp(29*x) + 264513735926050190573187366912*eu0^2*r0*x*exp(9*x) + 784080272382377731614232805376*eu0*r0*x^2*exp(19*x) + 2938033901373870616621048922112*eu0*r0*x*exp(19*x)), x, 0, t, 'IgnoreSpecialCases', true)/(30136102831353476067075555328*(eu0 + 10*exp(10*t))^(7058659091268371/1338311809433600)

%original%((exp(-991*x)*exp(-27/4)*exp(-3*x^2)*(184830*r0*exp(2000*x) - 3546*eu0^2*r0 + 2364*eu0^2*r0*x - 28695*eu0*r0*exp(1000*x) - 449760*r0*x*exp(2000*x) + 202680*r0*x^2*exp(2000*x) + 26380*ev0*exp(1981*x)*exp(27/4)*exp(3*x^2) + 20268*eu0*r0*x^2*exp(1000*x) - 13524*eu0*r0*x*exp(1000*x) + 3940*eu0*ev0*exp(981*x)*exp(27/4)*exp(3*x^2)))/(394*(eu0 + 10*exp(1000*x))^(196349/197000)))/(eu0 + 10*exp(1000*t))^(651/197000);
integrand_mf = matlabFunction( integrand );

%% building a trigonometric representation of stupid
pts = 1:1:4;
trigr0 = [];
trigt = [];
trigeu0 = [];
trigev0 = [];
for i = 1:length( pts )
    trigr0 = [ trigr0 sin( pts( i ) * r0 ) cos( pts( i ) * r0 ) ];
    trigt = [ trigt sin( pts( i ) * t ) cos( pts( i ) * t ) ];
    trigeu0 = [ trigeu0 sin( pts( i ) * eu0 ) cos( pts( i ) * eu0 ) ];
    trigev0 = [ trigev0 sin( pts( i ) * ev0 ) cos( pts( i ) * ev0 ) ];
end

basis = kron( kron( trigr0, trigt ), kron( trigeu0, trigev0 ) );

basis = basis( : );
basis_mf = matlabFunction( transpose( basis ) );

% to build the representation in our basis, we need to evaluate vdt_solved
r0_space = -0.2:0.05:0.2;
t_space = 0:.1:5;
eu_space = -2:.5:2;
ev_space = -0.1:0.05:0.1;
[ r0x, ty, eu0w, ev0z ] = ndgrid( r0_space, t_space, eu_space, ev_space );
eval_vdt_solved = zeros( size( ty( : ) ) );
for i= 1:length( ty( : ) )
    ds = linspace( 0, ty( i ), 1000 );
    eval_integrand = @(x) integrand_mf( eu0w(i), ev0z(i), r0x(i), ty(i), x );
    eval_vdt_solved( i ) = sum( eval_integrand( ds ) ) * ( ds( 2 ) - ds( 1 ) );
end

basis_eval = basis_mf(eu0w( : ), ev0z( : ),  r0x( : ), ty( : ) );

% this is running sparse regression
coeffs = lasso( basis_eval, eval_vdt_solved( : ), 'Lambda', 1e-3 );

% building the representation of vd(t) in our basis set
vdt_rep = matlabFunction( coeffs' * basis );

%% plots and viz
integrad_plot = feval(@(t)integrand_mf(eu_space(1), ev_space(1), r0_space(1),t,t),t_space)
figure(1); clf;hold on;
 plot(eval_vdt_solved);
 plot(t_space,vdt_rep(t_space));
