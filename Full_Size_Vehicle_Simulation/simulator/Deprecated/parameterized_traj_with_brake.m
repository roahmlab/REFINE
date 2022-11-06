function [T,U,Z]=parameterized_traj_with_brake(u0, v0, ud, r0)
% tpk given a time for a velocity to be reached at 
% tf automatically determined by current velocity
% u0 initial velocity
% ud desired velocity
% r0 magnitude of lane change, achieve gaussian heading change
% output ud and vd to car dynamics
amax = 3;
t_turn = 3.25;
% 
tf= ud/amax+t_turn;
Z=[];
% this is only fitted for new car constants and u = 10 +- 2  ev +- 0.1, %
% limited to gaussian lane change heading change peak at 1.5 sec. 
% give it 3.25 sec to finish lane change and straighten out, time fitted up
% to 5 s
% vdt_rep = @(eu0,ev0,r0,t)cos(eu0.*2.0).*cos(ev0).*cos(t).*sin(r0).*(-3.495264987167682e-1)+cos(t.*2.0).*cos(eu0).*cos(ev0).*sin(r0).*8.194970988649967e-2+sin(r0.*2.0).*cos(eu0).*cos(ev0).*cos(t).*9.799900813273343e-2+cos(eu0.*3.0).*cos(ev0).*cos(t).*sin(r0).*1.875279146410621e-1-cos(t.*3.0).*cos(eu0).*cos(ev0).*sin(r0).*5.711833353539638e-1+sin(r0.*3.0).*cos(eu0).*cos(ev0).*cos(t).*1.665815284513854e-2-cos(eu0.*4.0).*cos(ev0).*cos(t).*sin(r0).*3.33636291500067e-2-cos(t.*4.0).*cos(eu0).*cos(ev0).*sin(r0).*1.041022016608984e-1+sin(r0.*4.0).*cos(eu0).*cos(ev0).*cos(t).*7.677412034055802e-3+cos(eu0.*2.0).*cos(ev0).*sin(r0).*sin(t).*1.185942629650796e-2-sin(r0.*2.0).*cos(eu0).*cos(ev0).*sin(t).*7.755388470183353e-3+sin(t.*2.0).*cos(eu0).*cos(ev0).*sin(r0).*5.763301603828274e-1-cos(eu0.*3.0).*cos(ev0).*sin(r0).*sin(t).*5.081697072858528e-3-sin(r0.*3.0).*cos(eu0).*cos(ev0).*sin(t).*6.576009886025175e-3+sin(t.*3.0).*cos(eu0).*cos(ev0).*sin(r0).*5.008705501902104e-2-sin(r0.*4.0).*cos(eu0).*cos(ev0).*sin(t).*3.472005006981731e-3-sin(t.*4.0).*cos(eu0).*cos(ev0).*sin(r0).*4.145470466331707e-1+cos(eu0.*4.0).*cos(ev0.*2.0).*cos(t.*3.0).*sin(r0.*2.0).*4.150524117292899e-6+cos(eu0.*4.0).*cos(ev0.*2.0).*cos(t.*3.0).*sin(r0.*3.0).*2.835435362538372e-6+cos(eu0.*4.0).*cos(ev0.*2.0).*cos(t.*3.0).*sin(r0.*4.0).*2.200224800411791e-6-cos(eu0.*4.0).*cos(ev0.*2.0).*sin(r0.*2.0).*sin(t.*2.0).*4.479047329948215e-6-cos(eu0.*4.0).*cos(ev0.*2.0).*sin(r0.*3.0).*sin(t.*2.0).*3.059873769968427e-6+cos(eu0.*4.0).*cos(ev0.*2.0).*sin(r0.*2.0).*sin(t.*4.0).*2.549813260081415e-6-cos(eu0.*4.0).*cos(ev0.*2.0).*sin(r0.*4.0).*sin(t.*2.0).*2.374403842344229e-6+cos(eu0.*4.0).*cos(ev0.*2.0).*sin(r0.*3.0).*sin(t.*4.0).*1.741839264324556e-6+cos(eu0.*4.0).*cos(ev0.*2.0).*sin(r0.*4.0).*sin(t.*4.0).*1.351472065810618e-6-cos(eu0.*2.0).*cos(t.*2.0).*cos(ev0).*sin(r0).*6.489224725346769e-2-cos(eu0.*2.0).*sin(r0.*2.0).*cos(ev0).*cos(t).*4.901885112493687e-2+cos(t.*2.0).*sin(r0.*2.0).*cos(eu0).*cos(ev0).*2.042265075668121e-2+cos(eu0.*2.0).*cos(t.*3.0).*cos(ev0).*sin(r0).*4.790616537458128e-1-cos(eu0.*2.0).*sin(r0.*3.0).*cos(ev0).*cos(t).*1.278596089471576e-2+cos(eu0.*3.0).*cos(t.*2.0).*cos(ev0).*sin(r0).*2.639095705501824e-2+cos(eu0.*3.0).*sin(r0.*2.0).*cos(ev0).*cos(t).*2.610740303974822e-2+cos(t.*2.0).*sin(r0.*3.0).*cos(eu0).*cos(ev0).*6.354627178783214e-5-cos(t.*3.0).*sin(r0.*2.0).*cos(eu0).*cos(ev0).*1.367433065683142e-1+cos(eu0.*2.0).*cos(t.*4.0).*cos(ev0).*sin(r0).*8.353526574634082e-2-cos(eu0.*2.0).*sin(r0.*4.0).*cos(ev0).*cos(t).*4.859590059421807e-3-cos(eu0.*3.0).*cos(t.*3.0).*cos(ev0).*sin(r0).*2.644479640192852e-1+cos(eu0.*3.0).*sin(r0.*3.0).*cos(ev0).*cos(t).*8.83848777362177e-3-cos(eu0.*4.0).*cos(ev0.*2.0).*cos(t).*sin(r0).*4.317662993227849e-6-cos(eu0.*4.0).*sin(r0.*2.0).*cos(ev0).*cos(t).*1.869923697525358e-2+cos(t.*2.0).*sin(r0.*4.0).*cos(eu0).*cos(ev0).*1.300419982985446e-5-cos(t.*3.0).*sin(r0.*3.0).*cos(eu0).*cos(ev0).*2.996760346755416e-2-cos(t.*4.0).*sin(r0.*2.0).*cos(eu0).*cos(ev0).*3.101316127049831e-2-cos(eu0.*3.0).*cos(t.*4.0).*cos(ev0).*sin(r0).*3.826987649409277e-2+cos(eu0.*3.0).*sin(r0.*4.0).*cos(ev0).*cos(t).*2.199845004699073e-3+cos(eu0.*4.0).*cos(t.*3.0).*cos(ev0).*sin(r0).*6.315631926389162e-2-cos(eu0.*4.0).*sin(r0.*3.0).*cos(ev0).*cos(t).*2.923522147033524e-3-cos(t.*3.0).*sin(r0.*4.0).*cos(eu0).*cos(ev0).*9.958302762763235e-3-cos(t.*4.0).*sin(r0.*3.0).*cos(eu0).*cos(ev0).*1.048521199171651e-2+cos(eu0.*4.0).*cos(t.*4.0).*cos(ev0).*sin(r0).*2.261181879552755e-5-cos(eu0.*4.0).*sin(r0.*4.0).*cos(ev0).*cos(t).*9.265946953841524e-4-cos(t.*4.0).*sin(r0.*4.0).*cos(eu0).*cos(ev0).*4.262018955658944e-3+cos(eu0.*2.0).*sin(r0.*2.0).*cos(ev0).*sin(t).*6.688597276458943e-3-cos(eu0.*2.0).*sin(t.*2.0).*cos(ev0).*sin(r0).*4.8728184883823e-1+sin(r0.*2.0).*sin(t.*2.0).*cos(eu0).*cos(ev0).*1.215141881965745e-1+cos(eu0.*2.0).*sin(r0.*3.0).*cos(ev0).*sin(t).*3.534441401766013e-3-cos(eu0.*2.0).*sin(t.*3.0).*cos(ev0).*sin(r0).*3.689167666473485e-2-cos(eu0.*3.0).*sin(r0.*2.0).*cos(ev0).*sin(t).*7.160729092075134e-4+cos(eu0.*3.0).*sin(t.*2.0).*cos(ev0).*sin(r0).*2.701216063586797e-1+sin(r0.*3.0).*sin(t.*2.0).*cos(eu0).*cos(ev0).*1.43577533983985e-2+cos(eu0.*2.0).*sin(r0.*4.0).*cos(ev0).*sin(t).*1.916346725216374e-3+cos(eu0.*2.0).*sin(t.*4.0).*cos(ev0).*sin(r0).*3.482990891742488e-1-cos(eu0.*3.0).*sin(r0.*3.0).*cos(ev0).*sin(t).*2.778824715713977e-4+cos(eu0.*3.0).*sin(t.*3.0).*cos(ev0).*sin(r0).*2.013219030471585e-2-cos(eu0.*4.0).*sin(t.*2.0).*cos(ev0).*sin(r0).*6.811247198623224e-2-sin(r0.*2.0).*sin(t.*4.0).*cos(eu0).*cos(ev0).*1.087723905547456e-1+sin(r0.*4.0).*sin(t.*2.0).*cos(eu0).*cos(ev0).*1.633552104426915e-3-cos(eu0.*3.0).*sin(r0.*4.0).*cos(ev0).*sin(t).*1.108337753981902e-4-cos(eu0.*3.0).*sin(t.*4.0).*cos(ev0).*sin(r0).*1.877921368882409e-1-sin(r0.*3.0).*sin(t.*4.0).*cos(eu0).*cos(ev0).*2.574973643839878e-2+cos(eu0.*4.0).*sin(t.*4.0).*cos(ev0).*sin(r0).*3.77033122257132e-2-sin(r0.*4.0).*sin(t.*4.0).*cos(eu0).*cos(ev0).*8.070731167032458e-3+cos(eu0).*cos(ev0).*cos(t).*sin(r0).*4.132502909613978e-1-cos(eu0).*cos(ev0).*sin(r0).*sin(t).*2.513344449053179e-2-cos(eu0.*2.0).*cos(t.*2.0).*sin(r0.*2.0).*cos(ev0).*7.617919890217296e-3-cos(eu0.*2.0).*cos(t.*2.0).*sin(r0.*3.0).*cos(ev0).*7.250610120840974e-4+cos(eu0.*2.0).*cos(t.*3.0).*sin(r0.*2.0).*cos(ev0).*7.997096828513203e-2+cos(eu0.*3.0).*cos(t.*2.0).*sin(r0.*2.0).*cos(ev0).*3.081146301690478e-3-cos(eu0.*2.0).*cos(t.*2.0).*sin(r0.*4.0).*cos(ev0).*7.691771872001317e-5+cos(eu0.*2.0).*cos(t.*3.0).*sin(r0.*3.0).*cos(ev0).*2.036640861902747e-2+cos(eu0.*2.0).*cos(t.*4.0).*sin(r0.*2.0).*cos(ev0).*1.841358383782271e-2+cos(eu0.*3.0).*cos(t.*2.0).*sin(r0.*3.0).*cos(ev0).*4.082054954384696e-4-cos(eu0.*3.0).*cos(t.*3.0).*sin(r0.*2.0).*cos(ev0).*4.615753358117364e-2-cos(eu0.*4.0).*cos(ev0.*2.0).*sin(r0.*2.0).*cos(t).*2.190709865675697e-6+cos(eu0.*2.0).*cos(t.*3.0).*sin(r0.*4.0).*cos(ev0).*6.70724299032714e-3+cos(eu0.*2.0).*cos(t.*4.0).*sin(r0.*3.0).*cos(ev0).*5.936342575078191e-3+cos(eu0.*3.0).*cos(t.*2.0).*sin(r0.*4.0).*cos(ev0).*9.750125947391639e-6-cos(eu0.*3.0).*cos(t.*3.0).*sin(r0.*3.0).*cos(ev0).*1.41656393293852e-2-cos(eu0.*3.0).*cos(t.*4.0).*sin(r0.*2.0).*cos(ev0).*8.214825946636728e-3+cos(eu0.*4.0).*cos(ev0.*2.0).*cos(t.*3.0).*sin(r0).*8.180105025204746e-6-cos(eu0.*4.0).*cos(ev0.*2.0).*sin(r0.*3.0).*cos(t).*1.4964907962425e-6+cos(eu0.*4.0).*cos(t.*3.0).*sin(r0.*2.0).*cos(ev0).*2.685250165033599e-2+cos(eu0.*2.0).*cos(t.*4.0).*sin(r0.*4.0).*cos(ev0).*2.420502230893069e-3-cos(eu0.*3.0).*cos(t.*3.0).*sin(r0.*4.0).*cos(ev0).*4.010616483498305e-3-cos(eu0.*3.0).*cos(t.*4.0).*sin(r0.*3.0).*cos(ev0).*2.723848374393576e-3-cos(eu0.*4.0).*cos(ev0.*2.0).*sin(r0.*4.0).*cos(t).*1.161041500532223e-6+cos(eu0.*4.0).*cos(t.*3.0).*sin(r0.*3.0).*cos(ev0).*6.401912758193276e-3+cos(eu0.*4.0).*cos(t.*4.0).*sin(r0.*2.0).*cos(ev0).*4.911149255853453e-6-cos(eu0.*3.0).*cos(t.*4.0).*sin(r0.*4.0).*cos(ev0).*9.619000623977195e-4+cos(eu0.*4.0).*cos(t.*3.0).*sin(r0.*4.0).*cos(ev0).*1.788710537948799e-3+cos(eu0.*4.0).*cos(t.*4.0).*sin(r0.*3.0).*cos(ev0).*1.56352617782761e-6+cos(eu0.*4.0).*cos(t.*4.0).*sin(r0.*4.0).*cos(ev0).*8.097901961594619e-4-cos(eu0.*2.0).*sin(r0.*2.0).*sin(t.*2.0).*cos(ev0).*6.533086243625585e-2-cos(eu0.*2.0).*sin(r0.*3.0).*sin(t.*2.0).*cos(ev0).*1.166970806292366e-2+cos(eu0.*3.0).*sin(r0.*2.0).*sin(t.*2.0).*cos(ev0).*4.226651087784847e-2+cos(eu0.*2.0).*sin(r0.*2.0).*sin(t.*4.0).*cos(ev0).*6.427351215890768e-2-cos(eu0.*2.0).*sin(r0.*4.0).*sin(t.*2.0).*cos(ev0).*1.888574289307338e-3+cos(eu0.*3.0).*sin(r0.*3.0).*sin(t.*2.0).*cos(ev0).*1.067066650351839e-2-cos(eu0.*4.0).*cos(ev0.*2.0).*sin(t.*2.0).*sin(r0).*8.827580901161455e-6-cos(eu0.*4.0).*sin(r0.*2.0).*sin(t.*2.0).*cos(ev0).*2.750108661864541e-2+cos(eu0.*2.0).*sin(r0.*3.0).*sin(t.*4.0).*cos(ev0).*1.704596858938563e-2-cos(eu0.*3.0).*sin(r0.*2.0).*sin(t.*4.0).*cos(ev0).*3.627841330476916e-2+cos(eu0.*3.0).*sin(r0.*4.0).*sin(t.*2.0).*cos(ev0).*2.025003273437625e-3-cos(eu0.*4.0).*sin(r0.*3.0).*sin(t.*2.0).*cos(ev0).*5.922197010880269e-3+cos(eu0.*2.0).*sin(r0.*4.0).*sin(t.*4.0).*cos(ev0).*5.571798639325328e-3-cos(eu0.*3.0).*sin(r0.*3.0).*sin(t.*4.0).*cos(ev0).*1.156632137144307e-2+cos(eu0.*3.0).*sin(r0.*4.0).*sin(t.*3.0).*cos(ev0).*2.303500798162988e-5+cos(eu0.*4.0).*cos(ev0.*2.0).*sin(t.*4.0).*sin(r0).*5.025370308455265e-6+cos(eu0.*4.0).*sin(r0.*2.0).*sin(t.*4.0).*cos(ev0).*1.88418945177814e-2-cos(eu0.*4.0).*sin(r0.*4.0).*sin(t.*2.0).*cos(ev0).*1.362824207586281e-3-cos(eu0.*3.0).*sin(r0.*4.0).*sin(t.*4.0).*cos(ev0).*3.764709249363663e-3+cos(eu0.*4.0).*sin(r0.*3.0).*sin(t.*4.0).*cos(ev0).*5.837501244125833e-3+cos(eu0.*4.0).*sin(r0.*4.0).*sin(t.*4.0).*cos(ev0).*1.96347691990858e-3;


% v0 ~= 0; vd(t=0) = 0;
T = linspace(0,tf,1000);
idx = find(T<t_turn);
idx = idx(end);
% vd1 = vdt_rep(u0-ud, v0, r0, T(1:idx));
% ud1 = ud*ones(1,length(T(1:idx)))%this has to be 10
%new version of reference
A = r0;
t=T(1:idx);
ud1 = ud*ones(1,length(t));%this has to be 10
vd1 = A*exp(-3*(t - 3/2).^2).*(6*t - 9);
rd1 = -A/10*exp(-3*(t - 3/2).^2).*(6*t - 9);

ud2 = ud-amax.*(T(idx+1:end)-t_turn);
vd2 = zeros(1,1000-idx);
rd2 = zeros(1,1000-idx);


U = [ud1 ud2;vd1 vd2;rd1 rd2];
Z = cumsum(U,2)*(T(2)-T(1));



%%

% %%
% figure(11);clf;hold on;
% plot(T,F(1,:))
% % plot(t2+tpk,f2)
% figure(12);clf;hold on;
% plot(T(1:end-1),diff(F(1,:))./diff(T));
% 
% figure(13);clf;hold on;
% plot(T,F(2,:));


%this thing here cannot deal with dynamic where the first 0.5s the desired
%velocity needs to be something else, it has to be piecewise, tanh a better
%option.
%vx + (2*vx*t^3)/(tf - tpk)^3 - (vx*t^2*(6*tf - 6*tpk))/(2*(tf - tpk)^3)