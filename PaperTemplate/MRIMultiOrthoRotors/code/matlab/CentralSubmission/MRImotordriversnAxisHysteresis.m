function MRImotordriversnAxisHysteresis()
% Solution: use an event-type plot to switch using the hysteresis control
%
% switch(  1 to 0 if dV < alpha*dVmin
%          0 to 1 if dV > beta*dVmin
%         1>= alpha >= beta >=0   alpha = .8, beta = 0.2, dVmin = 50
%
% plots the position of n MRI rotors being controlled simultaneously
% to desired positions by an MRI gradient field. The MRI gradient field is
% defined by three (3) linear gradients in the x,y, and z axes.  This
% system is underactuated since 3 is much less than n. doing position
% control.
%
%   The rotors are modeled after the real, single -DOF actuators reported
%   in
%	P. Vartholomeos, C. Bergeles, L. Qin, P. E. and Dupont, "An MRI-powered
%	and Controlled Actuator Technology for Tetherless Robotic
%	Interventions", Int. J. Robotics Research, vol. 32, no. 13, pp.
%	1536-1552, 2013.
%   a pdf is available at:
%   http://robotics.tch.harvard.edu/publications/pdfs/vartholomeos2013MRI.pdf
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Author: Aaron Becker
%  Boston Children's Hospital
%  Date: 10/18/2013
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Setup workspace
%close all
format compact

set(0,'DefaultAxesFontSize',14)
set(0,'defaultaxesfontsize',14);
set(0,'DefaultTextInterpreter', 'latex')

MAKE_MOVIE = false;
%  Seed the generator
rng(10);

n=3; %number of rotors    %3 = 46 sec, 1- sim seconds
                          %10 = 25.573515 seconds, 60 sim seconds
                          %50 = 430.193182 seconds, 200 sim seconds

% CONSTANTS, section IV, IJRR article: http://robotics.tch.harvard.edu/publications/pdfs/vartholomeos2013MRI.pdf
steelSaturatedMag = 1.36*10^6;      % A/m  http://hyperphysics.phy-astr.gsu.edu/hbase/tables/magprop.html
maxMRIgrad = 0.040;                 %T/m  (page 4, IJRR)
steelBearingDiamInmm = 6;           %= [1,1.5,2,2.5,3,3.5,4,4.5,6,6.5,7,8,9,10,12,13,14,15,16,18,24]; % mm, bearing sizes available from McMaster
r = steelBearingDiamInmm/2*0.001;   % radius in m
Tfr =  5.0*10^-5;                   % 7*10^-5Nm    =  kg*m/(s^2) (friction PAGE 9, IJRR)
Tl = repmat(2.0*10^-5,1,n);         % load torque, Nm
Tl(randperm(n,floor(n/2))) = -Tl(1);
m = sphereVol(r)*8530;              %m^3* kg / (m^3), mass of bearing
r1 = 0.018;                         %m, radius rotor arm  with steel bearing.
J = 2*m*r1^2;                       % total inertia
b = 1.0*10^-7;                     %1.0*10^-7 Nms/rad     (PAGE 8, IJRR)
bearingVolMag = sphereVol(r)*steelSaturatedMag; %bearing volume* magnetic susceptibility

% SET INITIAL CONDITIONS
%Conventions: theta is angle of rotor
% phi is magnetic angle
% psi is slip angle
%state = [th1, th2, ..., thn, v1,v2, ... vn]

%initial velocity = 0m/s
state0 = zeros(2*n,1);
%initialize rotors to start, a random half at -100, the other half at 100 radians.
state0(1:n) = -250; %radians
state0(randperm(n, floor(n/2))) = 250;%radians

posDes = -state0(1:n);

Rs =  generateSpacedRotors(n);
u = zeros(3,n);
p = zeros(3,n);
r = zeros(3,n);
for i = 1:n  % Current position of rotor is Rs{i}*R_z(theta)*[1,0,0] (pointing to x-axis)
    u(:,i) = Rs{i}*[0;0;1]; %axis of rotation (nominally the z-axis)
    p(:,i) = Rs{i}*[0;1;0]; %perpendicular vector (nominally the point to y-axis)
    r(:,i) = Rs{i}*[1;0;0]; %rotor end (nominally the point to x-axis)
end


%%%% Simulate applying a constantly rotating control law to n rotors:
options = odeset('RelTol',1e-3,'AbsTol',1e-3);

tic
display(['Starting ODE: ',mfilename, ', ', datestr(clock)]);
tend = 5;
[T,Y] = ode45(@MRImultipleRotors,[0:tend/1000:tend],state0,options);
%[T,Y] = ode45(@MRImultipleRotors,[0,tend],state0,options);
toc

figure(5)
clf
set(gcf,'Name', 'Control Inputs and Lyapunov Function')
subplot(2,1,1)
PHIs = zeros(numel(T),3);
for i = 1:length(T)
    PHIs(i,:) = calcPhiVel(T(i),Y(i,:)')';
end
%plot(T,PHIs(:,1),T,PHIs(:,2),'.-',T,PHIs(:,3),':');
hC =stairs(T,[PHIs(:,1),PHIs(:,2),PHIs(:,3)]);
set(hC(2),'LineStyle','-.')
set(hC(3),'LineStyle',':')
legend('X','Y','Z','Location','east','Orientation','Horizontal');
ylabel({'gradient';'orientation'})


subplot(2,1,2)
% minimizes the candidate Lyapunov
        % function:  V() = sum ( 1/2 *(pos_{des} - pos_{act})^2 )
        %            Vdot() = sum( (pos_{des} - pos_{act})*Projection Of [Fx;Fy;Fz]
plot(T,sum((Y(:,1:n)-repmat(posDes(1:n)', size(T))).^2,2),'linewidth',2)

axis tight
a = axis;
axis([a(1),a(2),0,a(4)*1.05]);
xlabel('Time [s]')
ylabel('Error [rad$^2$]')
set(gcf,'papersize',[7,5])
set(gcf,'paperposition',[0,0,7,5])
%print -dpdf '../../pictures/pdf/PositionConverge40control2.pdf'

figure(50)
%TODO: plot Vdot
dV = zeros(3, numel(T));
for i = 1:numel(T)
    dV(:,i) = calcdV(T(i),Y(i,:));
end
plot(T,dV(1,:),T,dV(2,:),T,dV(3,:))
xlabel('Time [s]')
ylabel('dV')   %seems to indicate that abs(dV) < 50 isn't worth switching


figure(6)
clf
set(gcf,'Name', 'Lyapunov Function and Position Plots')
subplot(2,1,1)
plot(T,sum((Y(:,1:n)-repmat(posDes(1:n)', size(T))).^2,2),'linewidth',2)
axis tight
a = axis;
axis([a(1),a(2),0,a(4)*1.05]);
ylabel('Error [rad$^2$]')


subplot(2,1,2)
colors = repmat('r',n,1);
colors(posDes>0) = 'b';
for i = 1:n
plot(T,Y(:,i),'linewidth',1,'Color',colors(i))
hold on
end
axis tight
a = axis;
axis([a(1),a(2),a(3)*1.05,a(4)*1.05]);
xlabel('Time [s]')
ylabel('Position [rad]')
set(gcf,'papersize',[7,5])
set(gcf,'paperposition',[0,0,7,5])
%print -dpdf '../../pictures/pdf/PositionConverge40control.pdf'

display('finished')



    function v = sphereVol(r)
        v = 4/3*pi*r.^3;
    end

    function R = rotateAboutAxisTheta(u_x,u_y,u_z, theta)
        % rotation matrix R for rotating theta radians about axis
        % [ux,uy,yz]
        ctheta = cos(theta);
        stheta = sin(theta);
        R = [ ctheta + u_x^2*(1-ctheta) ,       u_x*u_y*(1-ctheta) - u_z*stheta ,   u_x*u_z*(1-ctheta) + u_y*stheta
            u_y*u_x*(1-ctheta) + u_z*stheta ,   ctheta + u_y^2*(1-ctheta) ,         u_y*u_z*(1-ctheta) - u_x*stheta
            u_z*u_x*(1-ctheta) - u_y*stheta ,   u_z*u_y*(1-ctheta) + u_x*stheta ,   ctheta + u_z^2*(1-ctheta)];
    end

    function Fout = staticFriction(Fapplied, FstaticFriction)
        % combines Static Friction and Applied Force
        Fout = 0;
        if  Fapplied > FstaticFriction
            Fout = Fapplied-FstaticFriction;
        elseif Fapplied < -FstaticFriction
            Fout = Fapplied+FstaticFriction;
        end
    end

    function Phi = calcPhiVel(~,Y)
        % minimizes the candidate Lyapunov
        % function:  V() = sum ( 1/2 *(vel_{des} - vel_{act})^2 )
        %            Vdot() = sum( (vel_{des} - vel_{act})*Projection Of [Fx;Fy;Fz]
        F = zeros(3,1);
        for ni = 1:n
            theta_i = Y(ni);
            vel_i = Y(n+ni);
            %calculate position error
            %gain of .1 gives very smooth, slow control of position.
            %gain of 5 is bumpy/overshoot
            gain = 1;  %this gain is almost meaningless.  It just needs to be positive
            posErr_i = gain*(posDes(ni) - theta_i); %^2*sign(posDes(ni) - theta_i);  
            % Todo:  an integral term would be nice here.
            velErr_i = posErr_i - vel_i;
            %compute perpendicular vector
            R = rotateAboutAxisTheta(u(1,ni),u(2,ni),u(3,ni), theta_i);
            pV = R * p(:,ni);
            %project force onto perpendicular vector
            % F = F+posErr_i*eye(3)*pV;  % works well with high viscous 
            F = F+velErr_i*eye(3)*pV;
        end
        Phi = sign(F);        
    end


function dV = calcdV(~,Y)
        % minimizes the candidate Lyapunov
        % function:  V() = sum ( 1/2 *(vel_{des} - vel_{act})^2 )
        %            Vdot() = sum( (vel_{des} - vel_{act})*Projection Of [Fx;Fy;Fz]
        dV = zeros(3,1);
        for ni = 1:n
            theta_i = Y(ni);
            vel_i = Y(n+ni);
            %calculate position error
            %gain of .1 gives very smooth, slow control of position.
            %gain of 5 is bumpy/overshoot
            gain = 1;  %this gain is almost meaningless.  It just needs to be positive
            posErr_i = gain*(posDes(ni) - theta_i); %^2*sign(posDes(ni) - theta_i);  
            % Todo:  an integral term would be nice here.
            velErr_i = posErr_i - vel_i;
            %compute perpendicular vector
            R = rotateAboutAxisTheta(u(1,ni),u(2,ni),u(3,ni), theta_i);
            pV = R * p(:,ni);
            %project force onto perpendicular vector
            % F = F+posErr_i*eye(3)*pV;  % works well with high viscous 
            dV = dV+velErr_i*eye(3)*pV;
        end       
    end


    function dy = MRImultipleRotors(t,y)
        % updates the differential equations for multiple rotors (this
        % function is called by ODE solver)
        PHI = calcPhiVel(t,y);
        %forces on balls
        F = bearingVolMag*maxMRIgrad*PHI;
        
        dy = zeros(size(y));    % a column vector
        for ni = 1:n  %TODO: unroll this
            theta_i = y(ni);
            vel_i = y(n+ni);
            %NOMINAL rotor rotates about the axis [0,0,1], for theta = 0 ferrite at [1,0,0];
            % ith rotor rotates about axis specified by unit vector u_i, to
            % angle theta_i
            %Step 1: rotate theta_i around axis
            R = rotateAboutAxisTheta(u(1,ni),u(2,ni),u(3,ni), theta_i);
            %compute perpendicular vector
            pV = R*p(:,ni);
            %project force onto perpendicular vector
            F_i = F'*pV;
            %dynamic equations
            dy(ni)   = y(n+ni);
            dy(n+ni) = 1/J*(  -b*vel_i -Tl(ni) +staticFriction(r1*F_i,Tfr) );
        end
        
    end

    function updateDrawing
        drawnow
        if(MAKE_MOVIE)
            FrameCount=FrameCount+1;
            figure(f2)
            tfig = myaa(4);
          %  F = getframe_nosteal_focus; %
            F = getframe;
            writeVideo(writerObj,F.cdata);
            close(tfig)
            while(FrameCount < 10)
                updateDrawing
            end
            
        end
    end

end




