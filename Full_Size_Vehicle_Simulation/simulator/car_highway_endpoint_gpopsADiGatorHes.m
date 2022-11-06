% This code was generated using ADiGator version 1.4
% ©2010-2014 Matthew J. Weinstein and Anil V. Rao
% ADiGator may be obtained at https://sourceforge.net/projects/adigator/ 
% Contact: mweinstein@ufl.edu
% Bugs/suggestions may be reported to the sourceforge forums
%                    DISCLAIMER
% ADiGator is a general-purpose software distributed under the GNU General
% Public License version 3.0. While the software is distributed with the
% hope that it will be useful, both the software and generated code are
% provided 'AS IS' with NO WARRANTIES OF ANY KIND and no merchantability
% or fitness for any purpose or application.

function output = car_highway_endpoint_gpopsADiGatorHes(input)
global ADiGator_car_highway_endpoint_gpopsADiGatorHes
if isempty(ADiGator_car_highway_endpoint_gpopsADiGatorHes); ADiGator_LoadData(); end
Gator1Data = ADiGator_car_highway_endpoint_gpopsADiGatorHes.car_highway_endpoint_gpopsADiGatorHes.Gator1Data;
Gator2Data = ADiGator_car_highway_endpoint_gpopsADiGatorHes.car_highway_endpoint_gpopsADiGatorHes.Gator2Data;
% ADiGator Start Derivative Computations
q.dv = input.phase.finalstate.dv;
q.f = input.phase.finalstate.f(:);
%User Line: q  = input.phase.finalstate(:);
waypt.dv = input.parameter.dv;
waypt.f = input.parameter.f(:);
%User Line: waypt = input.parameter(:);
cada1f1dv = q.dv(Gator1Data.Index2);
cada1f1 = q.f(Gator1Data.Index1);
cada1f2dv = waypt.dv(Gator1Data.Index4);
cada1f2 = waypt.f(Gator1Data.Index3);
cada1td1 =  zeros(4,1);
cada1td1(Gator1Data.Index5) = cada1f1dv;
cada2f1 = cada1td1(Gator1Data.Index6);
cada2f2 = uminus(cada1f2dv);
cada2f3 = cada2f1 + cada2f2;
cada1td1(Gator1Data.Index6) = cada2f3;
cada1f3dv = cada1td1;
cada1f3 = cada1f1 - cada1f2;
cada1tf2dv = cada1f3dv(Gator2Data.Index1);
cada1tf2 = cada1f3(Gator1Data.Index7);
cada2f1dv = cada1tf2dv;
cada2f1 = cada1tf2(:);
cada2tf2 = cada2f1(Gator2Data.Index2);
cada2f2dv = 1.*cada2tf2(:).^(1-1).*cada2f1dv;
cada2f2 = cada2f1.^1;
cada2f3dv = 2.*cada2f2dv;
cada2f3 = 2*cada2f2;
cada2tf1 = cada1f3dv(Gator2Data.Index3);
cada1f4dvdv = cada2tf1(:).*cada2f3dv;
cada1f4dv = cada2f3.*cada1f3dv;
cada1f4 = cada1f3.^2;
output.objective.dvdv = cada1f4dvdv; output.objective.dv = cada1f4dv;
output.objective.f = sum(cada1f4);
%User Line: output.objective = sum((q(1:2) - waypt(1:2)).^2);
output.objective.dv_size = 36;
output.objective.dv_location = Gator1Data.Index8;
output.objective.dvdv_size = [output.objective.dv_size,36];
output.objective.dvdv_location = [output.objective.dv_location(Gator2Data.Index4,:), Gator2Data.Index5];
end


function ADiGator_LoadData()
global ADiGator_car_highway_endpoint_gpopsADiGatorHes
ADiGator_car_highway_endpoint_gpopsADiGatorHes = load('car_highway_endpoint_gpopsADiGatorHes.mat');
return
end