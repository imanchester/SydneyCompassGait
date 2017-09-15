%----------------------------------------------------------------------------------------------
%----------------------------------------------------------------------------------------------
%------------------- Virtual constraint design using a Bezier polynomial ----------------------
%----------------------------------------------------------------------------------------------
%----------------------------------------------------------------------------------------------
% 
% This script can be used to design the reference trajectory for the hip joint using a Bezier 
% polynomial. 
% 
% For more information, please refer to: 
% """""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
% ""                           The Sydney Compass Gait Robot:                                ""
% ""        Design and Modeling of an Open Platform for Dynamic Walking Research             ""
% """""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
% 
% 
% Date created: 01/09/2016            Date modified: 14/09/2017
% Author: A. Mounir Boudali  
%----------------------------------------------------------------------------------------------
%----------------------------------------------------------------------------------------------
disp('*********************************************************************');

AngrangeA = 180/15; % ankle angle at impact --> "Step length"
A = 2*AngrangeA*[ -1.0, -0.9, -1.6, 3.3, 1.1, 0.95 ];% Bezier weighting points. The number of control points is the order of the bezier polynomial
disp(['VC design using ', num2str(length(A)), 'th order polynomial']);

n=size(A,2)-1; % set up codomain
one_ntab=0:1/n:1;
N=100;
s=0:1/(N-1):1;

C1=zeros(2,N);% initialization of C1
for k=0:n
    C1(1,:) = C1(1,:) + one_ntab(k+1)*s.^(k).*(1-s).^(n-k)*gamma(n+1)/(gamma(n-k+1)*gamma(k+1));
    C1(2,:) = C1(2,:) +        A(k+1)*s.^(k).*(1-s).^(n-k)*gamma(n+1)/(gamma(n-k+1)*gamma(k+1));
end

figure(1);
plot((0.5-one_ntab)*2*AngrangeA,A,'*'); hold on; grid on;
plot((0.5-s)*2*AngrangeA, C1(2,:), 'Linewidth', 1.5);
title('Virtual constraint');
xlabel('q_2');
ylabel('Desired trajectory: q_1r');

