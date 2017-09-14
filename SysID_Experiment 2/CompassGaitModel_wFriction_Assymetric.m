function [ dx,y ] = compassModel_friction_assymetric(t,x,innputUIO,m_inn,m_out,mH,mHg,I_c_inn, I_c_out, l, l_c_inn, l_c_out ,g0,k,Fc, Fv, vargin)
%----------------------------------------------------------------------------------------------
%----------------------------------------------------------------------------------------------
%----- System identification of an asymmetric compass gait walker from experimental data ------
%----------------------------------------------------------------------------------------------
%----------------------------------------------------------------------------------------------
% 
% This script is adapted from Dr. Justin Z. Tang's script and it represents an asymmetric 
% compass gait model with the presence of friction at the hip joint.
% 
% For more information, please refer to: 
% """""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
% ""                           The Sydney Compass Gait Robot:                                ""
% ""        Design and Modeling of an Open Platform for Dynamic Walking Research             ""
% """""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
% 
% 
% Date created: 02/08/2017          Date modified: 14/09/2017
% Authors: A. Mounir Boudali and Justin Z. Tang 
%----------------------------------------------------------------------------------------------
%----------------------------------------------------------------------------------------------
%% output equations
y = [x(1);x(2)]; % swing; stance

%% Definition of the state vector, control input, and switching signal
u      = innputUIO(1);  % control input
innout = innputUIO(2);  % which leg is the stance leg
q1     = x(1);          % hip angle
q2     = x(2);          % stance angle
q1Dot  = x(3);          % hip velocity
q2Dot  = x(4);          % stance velocity
theta_dot(1,1) = q1Dot; 
theta_dot(2,1) = q2Dot; 

% ASYMETRIC model: using "innout" to switch between inner/outer leg stance
H = [ innout*(m_inn*l_c_inn^2 + I_c_out) + (1-innout)*(m_out*l_c_out^2 + I_c_inn)                                                    , innout*(- m_inn*l_c_inn^2 + l*m_inn*cos(q1)*l_c_inn - I_c_out) + (1-innout)*(- m_out*l_c_out^2 + l*m_out*cos(q1)*l_c_out - I_c_inn)
innout*( - m_inn*l_c_inn^2 + l*m_inn*cos(q1)*l_c_inn - I_c_out) + (1-innout)*( - m_out*l_c_out^2 + l*m_out*cos(q1)*l_c_out - I_c_inn)    , innout*(I_c_out + I_c_inn + l^2*mH + l^2*m_out + l^2*m_inn + l_c_out^2*m_out + l_c_inn^2*m_inn - 2*l*l_c_out*m_out - 2*l*l_c_inn*m_inn*cos(q1)) + (1-innout)*(I_c_inn + I_c_out + l^2*mH + l^2*m_inn + l^2*m_out + l_c_inn^2*m_inn + l_c_out^2*m_out - 2*l*l_c_inn*m_inn - 2*l*l_c_out*m_out*cos(q1))] ;

C = [                                             0                                                        , innout*(-l*l_c_inn*m_inn*q2Dot*sin(q1)) + (1-innout)*(-l*l_c_out*m_out*q2Dot*sin(q1))
 innout*(-l*l_c_inn*m_inn*sin(q1)*(q1Dot - q2Dot)) + (1-innout)*(-l*l_c_out*m_out*sin(q1)*(q1Dot - q2Dot))     ,  innout*(l*l_c_inn*m_inn*q1Dot*sin(q1)) + (1-innout)*(l*l_c_out*m_out*q1Dot*sin(q1))];

G = [ innout*(g0*l_c_inn*m_inn*sin(q1 - q2)) + (1-innout)*(g0*l_c_out*m_out*sin(q1 - q2))
 innout*(-g0*(m_inn*(l*sin(q2) + l_c_inn*sin(q1 - q2)) + l*mHg*sin(q2) + m_out*sin(q2)*(l - l_c_out))) + (1-innout)*(-g0*(m_out*(l*sin(q2) + l_c_out*sin(q1 - q2)) + l*mHg*sin(q2) + m_inn*sin(q2)*(l - l_c_inn)))];

B = [k; 0];
%% Friction model : Viscous and Coulomb friction: using a continuous model of friction
friction = [( Fc*(2*sigmf(theta_dot(1),[10000 0]) - 1) )   +  (Fv*theta_dot(1)); 0];

%% State_k+1 computation 
q_dot(1:2) = theta_dot;
q_dot(3:4) =  -H\(C*theta_dot + G - B*u + friction);

%% Output of the function
dx = q_dot';

end