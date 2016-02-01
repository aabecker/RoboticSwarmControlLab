function Simulate3axisNeedleDriver()
    % Simulates an MRI powered and controlled multi-degree of freedom needle
    % driver.  This needle driver is inspired by that in
    % Conner Walsh's thesis  (http://hdl.handle.net/1721.1/61613 ) with 4
    % motors, two for orienting the driver:  thetaX, thetaY, one for advancing
    % the needle: thetaNeedle, and one for gripping the needle.  Our design
    % skips the gripping (which can be replaced with a passive spring).
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
    
    %  TODO: make visulization
    %  add in the forces
    %  Size the spheres for the forces necessary
    
    % Setup workspace
    %close all
    format compact
    
    set(0,'DefaultAxesFontSize',14)
    set(0,'defaultaxesfontsize',14);
    set(0,'DefaultTextInterpreter', 'latex')
    
    MAKE_MOVIE = false;
    FrameCount = 0;
    %  Seed the generator
    rng(10);
    
    n=3;
    
    % CONSTANTS, section IV, IJRR article: http://robotics.tch.harvard.edu/publications/pdfs/vartholomeos2013MRI.pdf
    steelSaturatedMag = 1.36*10^6;      % A/m  http://hyperphysics.phy-astr.gsu.edu/hbase/tables/magprop.html
    maxMRIgrad = 0.040;                 %T/m  (page 4, IJRR)
    steelBearingDiamInmm = 12;           %= [1,1.5,2,2.5,3,3.5,4,4.5,6,6.5,7,8,9,10,12,13,14,15,16,18,24]; % mm, bearing sizes available from McMaster
    r = steelBearingDiamInmm/2*0.001;   % radius in m
    Tfr =  7.0*10^-5;                   % 7*10^-5Nm    =  kg*m/(s^2) (friction PAGE 9, IJRR)  (max torque we can produce is 9.8*10^-4)
    Tl = zeros(1,n);         % load torque, Nm

    m = sphereVol(r)*8530;              %m^3* kg / (m^3), mass of bearing
    r1 = 0.02;                          %m, radius rotor arm  with steel bearing.
    J = 2*m*r1^2;                       % total inertia
    b = 10.0*10^-7;                     %1.0*10^-7 Nms/rad     (PAGE 8, IJRR)
    massCarriage = 0.100;               %kg, page 03  -- mass of bearing is 0.007 Kg
    hoopRadius = 0.05;                  % 50mm
    bearingVolMag = sphereVol(r)*steelSaturatedMag; %bearing volume* magnetic susceptibility
    GearRatioXY = 250;  %
    GearRatioNeedle = 250;
    RollerRadius = 0.005;%(pg 92) roller diameter = 10mm, they need 50Nmm for the friction drive
    RotorEfficiency = 0.5; % how much of torque is applied at exit
    
    % SET INITIAL CONDITIONS
    %state = [thX, thY, thNeedle, vX,vY, vNeedle]
    %initial velocity = 0m/s
    state0 = zeros(1,2*n);
 
    posDesA = [pi/6*GearRatioXY,-pi/6*GearRatioXY,0*GearRatioNeedle;  %position 1: needles
        pi/6*GearRatioXY,-pi/6*GearRatioXY,-0.050/(2*pi*RollerRadius)*GearRatioNeedle;
        pi/6*GearRatioXY,-pi/6*GearRatioXY,0*GearRatioNeedle;
        0,0,0;
        ]';  % insert 100mm
    
    Rs{1} = RxTheta(-pi/2); %rotate about the y-axis
    Rs{2} = RyTheta(pi/2)*RzTheta(pi/2); %rotate about the x-axis
    Rs{3} = eye(3);  %Nominally rotate about the z-axis  TODO: convert this to time-vary.
    
    u = zeros(3,n);
    p = zeros(3,n);
    r = zeros(3,n);
    for i = 1:n  % Current position of rotor is Rs{i}*R_z(theta)*[1,0,0] (pointing to x-axis)
        u(:,i) = Rs{i}*[0;0;1]; %axis of rotation (nominally the z-axis)
        p(:,i) = Rs{i}*[0;1;0]; %perpendicular vector (nominally pointing along the y-axis)
        r(:,i) = Rs{i}*[1;0;0]; %rotor end (nominally the point to x-axis)
    end
    
    figure(5)
    clf
    figure(6)
    clf
    %%%% Simulate applying position control law to n rotors:
    %TODO: increase this for more rotors.
    options = odeset('Events',@events,'RelTol',1e-3,'AbsTol',1e-3);
    tic
    display(['Starting ODE: ',mfilename, ', ', datestr(clock)]);
    tend = 120;  %total time alloted for procedure
    Ttot = 0;
    Ytot = state0;
    
    for posSel = 1:size(posDesA,2)
        posDes = posDesA(:,posSel);
        [T,Y] = ode45(@MRImultipleRotors,[Ttot(end),tend],Ytot(end,:),options);
        Ttot = [Ttot;T]; %#ok<*AGROW>
        Ytot = [Ytot;Y];
        plot_controlEffort_and_state()
        toc;
    end
    
    
    %  % method to incrementally do plotting.
    %     for tf = 1:numdiv
    %         %[Tnew,Ynew,TE,~,~] = ode45(@MRImultipleRotors,[(tf-1)/numdiv:1/1000:tf/numdiv]*tend,Y(end,:),options);
    %         [Tnew,Ynew,TE,~,~] = ode45(@MRImultipleRotors,[(tf-1)/numdiv,tf/numdiv]*tend,Y(end,:),options);
    %         %[T,Y] = ode45(@MRImultipleRotors,[0,tend],state0,options);
    %         T = [T;Tnew];
    %         Y = [Y;Ynew];
    %         plot_controlEffort_and_state()
    %         toc
    %         if numel(TE)>1
    %             break
    %         end
    %     end
    %
    %
    
    display('finished')
    
    function  plot_controlEffort_and_state()
        
        PHIs = zeros(numel(T),3);
        for k = 1:length(T)
            PHIs(k,:) = calcPhiVel(T(k),Y(k,:)')';
        end
        
        figure(5)
        set(gcf,'Name', 'Control Inputs and Lyapunov Function')
        subplot(2,1,1)
        hold on
        plot(T,PHIs(:,1),T,PHIs(:,2),'.-',T,PHIs(:,3),':');
        legend('X','Y','Z','Location','east','Orientation','Horizontal');
        ylabel({'gradient';'orientation'})
        
        subplot(2,1,2)
        hold on
        plot(T,sum((Y(:,1:n)-repmat(posDes(1:n)', size(T))).^2,2),'linewidth',2)
        axis tight
        a = axis;
        axis([a(1),a(2),0,a(4)*1.05]);
        xlabel('Time [s]')
        ylabel('Error [rad$^2$]')
        set(gcf,'papersize',[7,5])
        set(gcf,'paperposition',[0,0,7,5])
        %print -dpdf '../../pictures/pdf/PositionConverge40control2.pdf'
        
        figure(6)
        hold on
        set(gcf,'Name', 'Lyapunov Function and Position Plots')
        subplot(2,1,1)
        hold on
        plot(T,sum((Y(:,1:n)-repmat(posDes(1:n)', size(T))).^2,2),'linewidth',2)
        axis tight
        a = axis;
        axis([a(1),a(2),0,a(4)*1.05]);
        ylabel('Error [rad$^2$]')
        
        subplot(2,1,2)
        hold on
        colorOrder = [1,3,2];
        plot(T,Y(:,colorOrder),'linewidth',2) %arranged to get the colors right
        hold on
        plot([min(T),max(T)], [posDes(colorOrder),posDes(colorOrder)],'--','linewidth',1)
        
        axis tight
        a = axis;
        axis([a(1),a(2),a(3)*1.05,a(4)*1.05]);
        xlabel('Time [s]')
        ylabel('Position [rad]')
        set(gcf,'papersize',[7,5])
        set(gcf,'paperposition',[0,0,7,5])
        %print -dpdf '../../pictures/pdf/PositionConverge40control.pdf'
        print -dpdf '../../pictures/pdf/ThreeAxisNeedleInsertionL.pdf'
        drawnow
    end
    
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
            gain = 5;  %this gain is almost meaningless.  It just needs to be positive
            posErr_i = gain*(posDes(ni) - theta_i); %^2*sign(posDes(ni) - theta_i);
            % Todo:  an integral term would be nice here.
            
            %saturate
            posErr_i = min(50,max(-50,posErr_i ));
            velErr_i = posErr_i- vel_i;
            %compute perpendicular vector
            R = rotateAboutAxisTheta(u(1,ni),u(2,ni),u(3,ni), theta_i);
            pV = R* p(:,ni);
            %project force onto perpendicular vector
            % F = F+posErr_i*eye(3)*pV;  % works well with high viscous
            F = F+velErr_i*eye(3)*pV;
        end
        Phi = sign(F);
    end
    
    function dy = MRImultipleRotors(t,y)
        % updates the differential equations for multiple rotors (this
        % function is called by ODE solver)
        
        %update the orientation of the 3rd rotor
        newRotation3 = RyTheta(y(2))*RxTheta(y(1))*Rs{3};
        u(:,3) = newRotation3*[0;0;1]; %axis of rotation (nominally the z-axis)
        p(:,3) = newRotation3*[0;1;0]; %perpendicular vector (nominally pointing along the y-axis)
        
        PHI = calcPhiVel(t,y);
        %forces on balls
        F = bearingVolMag*maxMRIgrad*PHI;
        
        dy = zeros(size(y));    % a column vector
        
        gravityTorque = [sin( y(1)/GearRatioXY)*hoopRadius*massCarriage*9.801/GearRatioXY;
                          sin( y(2)/GearRatioXY)*hoopRadius*massCarriage*9.801/GearRatioXY; 
                          0];
        needleTorque = [ -(y(3)*(2*pi*RollerRadius)/GearRatioNeedle)*(60/100)*[1/GearRatioXY;1/GearRatioXY];
                          10*RollerRadius/GearRatioNeedle ]; % 10 N of force (maximum) on needle
        % moving y(3) to depth of 0.1m is -0.1/(2*pi*RollerRadius)*GearRatioNeedle

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
            dy(n+ni) = 1/J*(  -b*vel_i  +staticFriction(RotorEfficiency*r1*F_i -Tl(ni) -sign(vel_i)*needleTorque(i) + gravityTorque(ni),Tfr) );
        end
        
    end

    function Rx = RxTheta(theta)
        Rx = [  1,           0,  0;
            0,  cos(theta), -sin(theta);
            0,  sin(theta), cos(theta)];
    end
    
    function Ry = RyTheta(theta)
        Ry = [  cos(theta), 0,  sin(theta);
            0,          1, 0;
            -sin(theta), 0, cos(theta)];
    end
    
    function Rz = RzTheta(theta)
        Rz = [  cos(theta), -sin(theta),  0;
            sin(theta),  cos(theta),  0;
            0,                    0,  1];
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
    
    function [value,isterminal,direction] = events(~,y)
        % Locate the time when we are within the desired position of our
        % goal.
        n = numel(y)/2;
        theta = y(1:n);
        %vels =  y(n+1:2*n);
        value = 180/pi*max(abs(posDes-theta))-45; %detect when maximum angle error<45 degrees
        isterminal = 1;   % stop the integration
        direction = 0;   % any direction
    end
end




