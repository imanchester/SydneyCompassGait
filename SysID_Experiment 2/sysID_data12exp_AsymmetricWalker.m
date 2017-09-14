%----------------------------------------------------------------------------------------------
%----------------------------------------------------------------------------------------------
%----- System identification of an asymmetric compass gait walker from experimental data ------
%----------------------------------------------------------------------------------------------
%----------------------------------------------------------------------------------------------
% 
% This script is adapted from Justin Z. Tang's script and it does the following tasks:
% 1/ Segments the data from a walking experiment using "DEMOSYSTEM.slx" in two data sets: 
%   -data when the innel leg is stance
%   -data when the outer leg is stance
% 2/ Records the pre/post impacts states
% 3/ Performs a non linear parameter estimation using a Asymmetric model of the compass gait 
%   -file: "compassModel_friction_assymetric"
% 4/ Plots the simulation of the identified model and the validation data
% 5/ Saves the figures 
% 
% For more information, please refer to: 
% """""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
% ""                          The Sydney Compass Gait Robot:                                 ""
% ""        Design and Modeling of an Open Platform for Dynamic Walking Research             ""
% """""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
% 
% 
% Date created: 02/08/2017          Date modified: 14/09/2017 
% Authors: A. Mounir Boudali and Justin Z. Tang 
%----------------------------------------------------------------------------------------------
%----------------------------------------------------------------------------------------------

clear all; tic;

%% Training data - using 3 VCs and 4 control strategies 
loadName = 'SysID_data_12exp_120steps.mat';% data from 12 diff experiments, 10 consecutive steps each, 120 steps
% Note: something is wrong with step 4 in the data after inn out separation 
load(loadName);
startingTimeIndex = 1; endingTimeIndex = 13788;% all data SysID_data_12exp_120steps
% startingTimeIndex = 1; endingTimeIndex = 3884;% 3VC, 1 ctrl strategy
deleteUpTo = 7; % Remove few data points post and pre impact, which represent vibrations

do_sysid = true;
plotRaw  = false;   % plot raw data (control, stance, swing)
plotall  = false;   % plot all raw data (state, control input, for inn out leg stance)
order    = [2,2,4]; % model orders [ny, nu, nx] 2 inputs, 2 outputs, and 4 states  
flexiparam   = 3;   % 1 no flexibility, 2 "flexipercent%" on id params from exp1, 3 no bounds
fp           = 0.5; % percentage flexibility  
fixinitposi  = 1;   % 0 fixes the parameter, 1 set it free 
fixinitvelo  = 0;   % 

%% Parameters: values and initial guess
Nm_per_V = 0.0182;  % from datasheet 
% Nm_per_V = 0.0201; % from single pendulum sysid
[g0, k] = deal(9.81, 1);% constants

%% Ic 0.1 for all
% [m_sw, l_c_sw, I_c_sw] = deal(0.7, 0.1, 0.1);% data from inner leg swing
% [m_st, l_c_st, I_c_st] = deal(0.7, 0.1, 0.1);
% [mH, mHg, l]           = deal(0.1, -0.409, 0.2778);
% [Fc, Fv]               = deal(0.1, 0.1);% friction
%% Ic are /r to COM - friction from average
[m_inn, l_c_inn] = deal(0.7, 0.0448);% data from inner leg swing
I_c_inn         = 0.00432375886090166-m_inn*l_c_inn^2;
[m_out, l_c_out] = deal(0.7, 0.0502);
I_c_out         = 0.00436943439848782-m_out*l_c_out^2;
[mH, mHg, l]   = deal(2, -0.409, 0.2778);
[Fc, Fv]       = deal(0.006110566742079, 0.001561351396977);% friction

initvalues = {m_inn, m_out, mH, mHg, I_c_inn, I_c_out, l, l_c_inn, l_c_out, g0, k, Fc, Fv};

%% Fixed/Free parameters : 
% fixed_param_cell_all = {true true false true false false true false false true true false false};% only "measured" fixed
% fixed_param_cell_all = {false false false false false false true false false true true false false};% only some of "measured" fixed
fixed_param_cell_all = {true true false true true true true true true true true true true};% only mh free: Experiment 2

%% 
disp('=====================================================================');

%% get states, plot raw data
torque_input     = -Nm_per_V*umaxonsatu;% Control input
stance_leg_angle = stanceangle;         % State: q2
swing_leg_angle  = deg2rad(hipangle);            % State: q1

    if plotRaw == 1 % plot the raw data
        figure(1);
        subplot(2,1,1);
        subplot(3,1,1); plot(datatime, torque_input);     ylabel('Control input');
        subplot(3,1,2); plot(datatime, stance_leg_angle); ylabel('Stance leg angle');
        subplot(3,1,3); plot(datatime, swing_leg_angle);  ylabel('Swing leg angle');
        subplot(2,1,2);
        plot(datatime, innoutst); ylabel('OuterLegStance');
    end

%% Mapping from walker data to model data
outerLegStanceData    = innoutst; %state.outerLegStance.data;
stance_leg_data_fixed = -stance_leg_angle;
swing_leg_data_fixed  = -2*(innoutst-0.5).*swing_leg_angle;
    
outer_enco        = -2*(innoutst-0.5).*swing_leg_data_fixed;
inner_enco        = -(innoutst-1).*swing_leg_data_fixed - stance_leg_data_fixed;
outer_enco_dot    = gradient(outer_enco, T);
inner_enco_dot    = gradient(inner_enco, T);
outer_enco_dotdot = gradient(outer_enco_dot, T);
inner_enco_dotdot = gradient(inner_enco_dot, T);

swing_leg_dot  = -2*(innoutst-0.5).*outer_enco_dot;
stance_leg_dot = -inner_enco_dot -(innoutst-1).*outer_enco_dot;
% swing_leg_dotdot  = -2*(innoutst-0.5).*outer_enco_dotdot; % acceleration
% stance_leg_dotdot = -inner_enco_dotdot -(innoutst-1).*outer_enco_dotdot;

%% Separating data: inner leg stance data & outer leg stance data
%% outer leg stance
periodCount        = 0;
lastOuterLegStance = 0;
collectOuterLeg    = true;

%% Impact map and initial states 
I_c = (I_c_inn+I_c_out)/2;
l_c = (l_c_inn+l_c_out)/2;
m   = (m_inn+m_out)/2;

for i = startingTimeIndex:endingTimeIndex
    if outerLegStanceData(i) ~= collectOuterLeg
        lastOuterLegStance = 0;
        continue
    end
    if outerLegStanceData(i) == collectOuterLeg
        if lastOuterLegStance == 0
            % new period!!
            periodCount  = periodCount+1;
            % store impact point and post impact point
            qMinusIndex  = i;
            qMinusSwing  = swing_leg_angle(qMinusIndex);  % do NOT use -1 here because last period was outerLegStance=false
            qMinusStance = -stance_leg_angle(qMinusIndex);
            qMinusAll{periodCount} = [qMinusSwing; qMinusStance];
            qPlusAll{periodCount}  = [-swing_leg_angle(deleteUpTo-1+i) ; ...
                                      -stance_leg_angle(deleteUpTo-1+i)]; % 
            q1 = qMinusSwing; 
            q2 = qMinusStance; 
            qDotMinusAll{periodCount} = [swing_leg_dot(qMinusIndex); stance_leg_dot(qMinusIndex)];
            deltaqDot = mappingdot(I_c, l, l_c, m, mH, q1, q2);
            qDotPlusAll{periodCount} = deltaqDot * qDotMinusAll{periodCount};
            %store new empty vector containing the stance/swing/control input /innout signal
            stanceAll{periodCount}     = [];
            swingAll{periodCount}      = [];
            torqueInputAll{periodCount}= [];
            innoutsigAll{periodCount}  = [];
        end

        % store into struct
        currentPeriodStance         = stanceAll{periodCount};
        currentPeriodStance(end+1)  = -stance_leg_angle(i);
        stanceAll{periodCount}      = currentPeriodStance;

        currentPeriodSwing          = swingAll{periodCount};
        currentPeriodSwing(end+1)   = -swing_leg_angle(i);
        swingAll{periodCount}       = currentPeriodSwing;

        currentTorqueInput          = torqueInputAll{periodCount};
        currentTorqueInput(end+1)   = torque_input(i);
        torqueInputAll{periodCount} = currentTorqueInput;

        currentInnoutInput          = innoutsigAll{periodCount};
        currentInnoutInput(end+1)   = innoutst(i);
        innoutsigAll{periodCount}   = currentInnoutInput;

        lastOuterLegStance = 1;
    end
end

% Define which periods to use 
    plotPeriods_inSt = 2:periodCount-2;
    plotPeriods_outerSt = 2:periodCount-2;

    for i = plotPeriods_outerSt
        currentPeriodStance = stanceAll{i};
        currentPeriodStance = currentPeriodStance(deleteUpTo:end-deleteUpTo);
        stanceAll{i}        = currentPeriodStance;

        currentPeriodSwing  = swingAll{i};
        currentPeriodSwing  = currentPeriodSwing(deleteUpTo:end-deleteUpTo);
        swingAll{i}         = currentPeriodSwing;

        currentTorqueInput  = torqueInputAll{i};
        currentTorqueInput  = currentTorqueInput(deleteUpTo:end-deleteUpTo);
        torqueInputAll{i}   = currentTorqueInput;
        
        currentInnoutInput  = innoutsigAll{i};
        currentInnoutInput  = currentInnoutInput(deleteUpTo:end-deleteUpTo);
        innoutsigAll{i}     = currentInnoutInput;
    end

% store into iddata
expData = iddata([swingAll{plotPeriods_outerSt(1)}', stanceAll{plotPeriods_outerSt(1)}'], ...
                 [torqueInputAll{plotPeriods_outerSt(1)}', innoutsigAll{plotPeriods_outerSt(1)}'], ...
                  T);
x1Plus_outerSt = [];
x2Plus_outerSt = [];
x1DotPlus_outerSt = [];
x2DotPlus_outerSt = [];
    for i = plotPeriods_outerSt 
        % store xPlus IC
        xPlus = qPlusAll{i};
        x1Plus_outerSt(end+1) = xPlus(1);
        x2Plus_outerSt(end+1) = xPlus(2);

        xDotPlus = qDotPlusAll{i};
        x1DotPlus_outerSt(end+1) = xDotPlus(1);
        x2DotPlus_outerSt(end+1) = xDotPlus(2);

        if i == plotPeriods_outerSt(1)
            continue
        end
        expData = merge(expData, ...
                        iddata( [swingAll{i}' , stanceAll{i}'], ...
                                [torqueInputAll{i}', innoutsigAll{i}'], ...
                                 T));% 
    end
% calculate velocity and plot
    for period = plotPeriods_outerSt
        stanceDotAll{period} = gradient(stanceAll{period},T);
        swingDotAll{period}  = gradient(swingAll{period},T);
    end

%% inner leg stance
periodCount_inSt   = 0;
lastOuterLegStance = 0;
collectOuterLeg    = false;

for i = startingTimeIndex:endingTimeIndex
    if outerLegStanceData(i) ~= collectOuterLeg
        lastOuterLegStance    = 0;
        continue
    end
    if outerLegStanceData(i)  == collectOuterLeg
        if lastOuterLegStance == 0
            % new period!!
            periodCount_inSt = periodCount_inSt+1;
            % store impact point and post impact point
            qMinusIndex       = i;
            qMinusSwing_inSt  = -swing_leg_angle(qMinusIndex);  % use -1 here because last period was outerLegStance=true 
            qMinusStance_inSt = -stance_leg_angle(qMinusIndex);
            qMinusAll_inSt{periodCount_inSt} = [qMinusSwing_inSt; qMinusStance_inSt];
            qPlusAll_inSt{periodCount_inSt}  = [swing_leg_angle(deleteUpTo-1+i) ; ...
                                                -stance_leg_angle(deleteUpTo-1+i)];
            q1 = qMinusSwing_inSt;
            q2 = qMinusStance_inSt;
            qDotMinusAll_inSt{periodCount_inSt} = [swing_leg_dot(qMinusIndex); stance_leg_dot(qMinusIndex)];
            deltaqDot = mappingdot(I_c, l, l_c, m, mH, q1, q2);
            qDotPlusAll_inSt{periodCount_inSt} = deltaqDot * qDotMinusAll_inSt{periodCount_inSt};
            % store new empty vector containing the stance/swing/control input/innout signal
            stanceAll_inSt{periodCount_inSt}      = [];
            swingAll_inSt{periodCount_inSt}       = [];
            torqueInputAll_inSt{periodCount_inSt} = [];
            innoutsigAll_inSt{periodCount_inSt}   = [];
        end

        % store into struct
        currentPeriodStance              = stanceAll_inSt{periodCount_inSt};
        currentPeriodStance(end+1)       = -stance_leg_angle(i);
        stanceAll_inSt{periodCount_inSt} = currentPeriodStance;

        currentPeriodSwing              = swingAll_inSt{periodCount_inSt};
        currentPeriodSwing(end+1)       = swing_leg_angle(i);
        swingAll_inSt{periodCount_inSt} = currentPeriodSwing;

        currentTorqueInput                    = torqueInputAll_inSt{periodCount_inSt};
        currentTorqueInput(end+1)             = -torque_input(i);
        torqueInputAll_inSt{periodCount_inSt} = currentTorqueInput;

        currentInnoutInput                  = innoutsigAll_inSt{periodCount_inSt};
        currentInnoutInput(end+1)           = innoutst(i);
        innoutsigAll_inSt{periodCount_inSt} = currentInnoutInput;

        lastOuterLegStance = 1;
    end 
end

    for i = plotPeriods_inSt
        currentPeriodStance    = stanceAll_inSt{i};
        currentPeriodStance    = currentPeriodStance(deleteUpTo:end-deleteUpTo);
        stanceAll_inSt{i}      = currentPeriodStance;

        currentPeriodSwing     = swingAll_inSt{i};
        currentPeriodSwing     = currentPeriodSwing(deleteUpTo:end-deleteUpTo);
        swingAll_inSt{i}       = currentPeriodSwing;

        currentTorqueInput     = torqueInputAll_inSt{i};
        currentTorqueInput     = currentTorqueInput(deleteUpTo:end-deleteUpTo);
        torqueInputAll_inSt{i} = currentTorqueInput;
        
        currentInnoutInput     = innoutsigAll_inSt{i};
        currentInnoutInput     = currentInnoutInput(deleteUpTo:end-deleteUpTo);
        innoutsigAll_inSt{i}   = currentInnoutInput;
    end

% calculate velocity and plot
    for period = plotPeriods_inSt
        stanceDotAll_inSt{period} = gradient(stanceAll_inSt{period},T);
        swingDotAll_inSt{period}  = gradient(swingAll_inSt{period},T);
    end

expDataAll_inSt = iddata([swingAll_inSt{plotPeriods_inSt(1)}', stanceAll_inSt{plotPeriods_inSt(1)}'], ...
                         [torqueInputAll_inSt{plotPeriods_inSt(1)}', innoutsigAll_inSt{plotPeriods_inSt(1)}'], ...
                          T);
x1Plus_inSt     = [];
x2Plus_inSt     = [];
x1DotPlus_inSt  = [];
x2DotPlus_inSt  = [];

    for i = plotPeriods_inSt
        % store xPlus IC
        xPlus              = qPlusAll_inSt{i};
        x1Plus_inSt(end+1) = xPlus(1);
        x2Plus_inSt(end+1) = xPlus(2);

        xDotPlus              = qDotPlusAll_inSt{i};
        x1DotPlus_inSt(end+1) = xDotPlus(1);
        x2DotPlus_inSt(end+1) = xDotPlus(2);

        if i == plotPeriods_inSt(1)
            continue
        end
        expDataAll_inSt = merge(expDataAll_inSt, ...
                                iddata([swingAll_inSt{i}' , stanceAll_inSt{i}'], ...
                                       [torqueInputAll_inSt{i}', innoutsigAll_inSt{i}'], ...
                                        T));
    end

%% System identification and parameter estimation 
totalExps = length([x1Plus_outerSt,x1Plus_inSt;  x2Plus_outerSt,x2Plus_inSt]);% total number of experiments

nonlinear_model = idnlgrey('compassModel_friction_assymetric',order,initvalues, ...
                            [x1Plus_outerSt,   x1Plus_inSt; ...
                             x2Plus_outerSt,   x2Plus_inSt; ...
                             x1DotPlus_outerSt,x1DotPlus_inSt; ...
                             x2DotPlus_outerSt,x2DotPlus_inSt]);

%% fix/free the initial angles and velocities
    nonlinear_model.InitialStates(1).Fixed = fixinitposi*ones(size([x1Plus_outerSt,x1Plus_inSt])); 
    nonlinear_model.InitialStates(2).Fixed = fixinitposi*ones(size([x1Plus_outerSt,x1Plus_inSt])); 
    nonlinear_model.InitialStates(3).Fixed = fixinitvelo*ones(size([x1Plus_outerSt,x1Plus_inSt])); 
    nonlinear_model.InitialStates(4).Fixed = fixinitvelo*ones(size([x1Plus_outerSt,x1Plus_inSt])); 

setpar(nonlinear_model, 'Fixed', fixed_param_cell_all);

if     flexiparam == 1 % bounds are min and max from experiment 1 
    %         {m_il, m_ol, mH, mHg, I_c_il, I_c_ol,  l, l_c_il, l_c_ol, g0,  k,  Fc,  Fv}
    Minvals = {0.7 0.7  0 -2 min(I_c_inn, I_c_out) min(I_c_inn, I_c_out) 0.25  0.0448 0.0448 9.80 0.85 0.00544338360765557 0.0014037855482214}; 
    Maxvals = {0.7 0.7 15  0 max(I_c_inn, I_c_out) max(I_c_inn, I_c_out) 0.30  0.0502 0.0502 9.82 1.15 0.00677774987650278 0.00171891724573325}; 
elseif flexiparam == 2 % allow 10% flexibility on identified parameters and some of the measured ones 
    Minvals = {m_inn*(1-fp) m_out*(1-fp)  0  mHg*(1+fp) I_c_inn*(1-fp)  I_c_out*(1-fp)  0.25  l_c_inn*(1-fp)  l_c_out*(1-fp)  9.80   0.85   Fc*(1-fp)  Fv*(1-fp)};
    Maxvals = {m_inn*(1+fp) m_out*(1+fp) 15  mHg*(1-fp) I_c_inn*(1+fp)  I_c_out*(1+fp)  0.30  l_c_inn*(1+fp)  l_c_out*(1+fp)  9.82   1.15   Fc*(1+fp)  Fv*(1+fp)};
elseif flexiparam == 3 % no bounds on identified parameters 
    Minvals = {-100 -100 -100 -100 -100 -100 -100 -100 -100 9.80 -100 -100 -100};
    Maxvals = { 100  100  100  100  100  100  100  100  100 9.82  100  100  100};
end

setpar(nonlinear_model,'Minimum',Minvals);
setpar(nonlinear_model,'Maximum',Maxvals);

expData_bothlegs = merge(expData , expDataAll_inSt); 

if do_sysid
    opt = nlgreyestOptions;
    opt.SearchOption.MaxIter = 100;
    opt.Display = 'on';
    nonlinear_iden = nlgreyest(expData_bothlegs, nonlinear_model, opt);
end

%% plot all raw data 
if plotall == 1
subplotx = 4; subploty = 2;

figure();
subplot(subplotx,subploty,2); hold on;
title('stance leg')
xlabel('time label')
ylabel('stance leg angle')
for period = plotPeriods_outerSt
    plot(stanceAll{period})
end

subplot(subplotx,subploty,1); hold on
title('swing leg')
xlabel('time label')
ylabel('swing leg angle')
for period = plotPeriods_outerSt
    plot(swingAll{period})
end

subplot(subplotx,subploty,4); hold on;
title('stance leg dot')
xlabel('time label')
ylabel('stance leg angle')
for period = plotPeriods_outerSt
    plot(stanceDotAll{period})
end

subplot(subplotx,subploty,3); hold on
title('swing leg dot')
xlabel('time label')
ylabel('swing leg angle')
for period = plotPeriods_outerSt
    plot(swingDotAll{period})
end

subplot(subplotx,subploty,6); hold on
for period = plotPeriods_outerSt
    plot(stanceAll{period},stanceDotAll{period})
end

subplot(subplotx,subploty,5); hold on
for period = plotPeriods_outerSt
    plot(swingAll{period},swingDotAll{period})
end

subplot(subplotx,subploty,7); hold on
title('input torque')
xlabel('time label')
ylabel('input torque')

for period = plotPeriods_inSt
    plot(torqueInputAll_inSt{period})
end
for period = plotPeriods_outerSt
    plot(torqueInputAll{period})
end
end

noExperiments  = length(expData.Experiment);
fixedParams    = getpar(nonlinear_iden,'Fixed');% record which parameters were fixed...
fixedParamsStr = '';
fixedParamsStr = num2str(cell2mat(fixedParams)'); fixedParamsStr = fixedParamsStr(fixedParamsStr~=' ');
%% Save the plots ...
if do_sysid
saveFolderName = [fixedParamsStr,'_',num2str(noExperiments),...
                'OUTER_STANCE_ASYM__Nm_per_V=',sprintf('%g',Nm_per_V),...
                '_new_MaxMinParam_deleteUpTo3_FREE_INIT_VELOCITY_NewFixAngle'];

if ~exist(saveFolderName,'dir')
    mkdir(saveFolderName)
end

% extract identified parameters 
param_id = zeros(1,length(initvalues));
for i = 1:length(nonlinear_iden.Parameters)
    param_id(i) = nonlinear_iden.Parameters(i).Value;
end
param_id_str = mat2str(param_id,3);

for iter = 1:length(expData_bothlegs.Experiment)
    ICx1 = nonlinear_iden.InitialStates(1).Value(iter);
    ICx2 = nonlinear_iden.InitialStates(2).Value(iter);
    ICx3 = nonlinear_iden.InitialStates(3).Value(iter);
    ICx4 = nonlinear_iden.InitialStates(4).Value(iter);
    
    opt = compareOptions('InitialCondition',[ICx1, ICx2, ICx3, ICx4]');

    figure_handle = figure();% creates one figure per step
    compare(getexp(expData_bothlegs, iter ) , nonlinear_iden, opt);
    [modelresp, fit_val, initcondi_x0] = compare(getexp(expData_bothlegs, iter), nonlinear_iden, opt);
    
    str_modelresp{iter}    = modelresp;
    str_fit_val{iter}      = fit_val;
    str_initcondi_x0{iter} = initcondi_x0;
    
    supTitleStr = sprintf('Exp%d/%d  I.C.=[%.4f, %.4f, %.4f, %.4f] ASYMMETRIC WALKER',iter,noExperiments, ICx1, ICx2, ICx3, ICx4); 
    titleString = sprintf('%s\n[m(ol),m(il),mH,mHg, I_c(ol),I_c(il), l,l_c(ol),l_c(il) ,g0,k,Fc,Fv] \nvalues=%s \nfixed=%s \n LossFcn=%g FinalPredictErr=%g ',...
                          supTitleStr, param_id_str, fixedParamsStr, nonlinear_iden.Report.Fit.LossFcn, nonlinear_iden.Report.Fit.FPE );
    title(titleString);
    legend('Location','Best')
    
    saveas(figure_handle, sprintf('%s/%sExp%dof%d.pdf',saveFolderName,fixedParamsStr,iter,noExperiments));
end
save(sprintf('%s/%s_%dexp_DATA.mat',saveFolderName,fixedParamsStr,noExperiments))
end

disp('Asymmetric Walker');
cellsvalues = ['fixed' fixed_param_cell_all; ...
                'init val' initvalues ; ...
                'min  val' Minvals; ...
                'id   val' num2cell(param_id); ...
                'max  val' Maxvals];

cell2table(cellsvalues, 'VariableNames', {'aaa' 'mol' 'mil' 'mH' 'mHg' 'Icol' 'Icil' 'l' 'lcol' 'lcil' 'g0' 'k' 'Fc' 'Fv'})

%% show the covariance matrix
disp('covar matrix: ')
diag(nonlinear_iden.covar)'

toc;% processing time for the system identification  

% ----------------------------------------------------------------------------------------------
% ----------------------------------------------------------------------------------------------