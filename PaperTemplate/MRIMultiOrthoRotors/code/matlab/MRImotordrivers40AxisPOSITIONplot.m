function MRImotordrivers40AxisPOSITIONplot()
    % plots 40 rotors being controlled simultaneously by MRI gradient field
    % doing position control
    
    %  Author: Aaron Becker
    %  Date: 10/18/2013
    %close all
    format compact
    clc
    set(0,'DefaultAxesFontSize',14)
    set(0,'defaultaxesfontsize',14);
    
    MAKE_MOVIE = false;
    FrameCount = 0;
    %  Seed the generator
    rng(10);
    % CONSTANTS, section IV, IJRR article
    steelSaturatedMag = 1.36*10^6;      % A/m  http://hyperphysics.phy-astr.gsu.edu/hbase/tables/magprop.html
    maxMRIgrad = 0.040;                 %T/m  (page 4, IJRR)
    steelBearingDiamInmm = 6;           %= [1,1.5,2,2.5,3,3.5,4,4.5,6,6.5,7,8,9,10,12,13,14,15,16,18,24]; % mm, bearing sizes available from McMaster
    r = steelBearingDiamInmm/2*0.001;   % radius in m
    Tfr =  8.0*10^-5;                   % 7*10^-5N    =  kg*m/(s^2) (friction PAGE 8, IJRR)
    Tl = 0;                              % load torque
    m = sphereVol(r)*8530;              %m^3* kg / (m^3), mass of bearing
    r1 = 0.018;                         %m, radius rotor arm  with steel bearing.
    J = 2*m*r1^2;                       % total inertia
    b = 10.0*10^-7;                     %1.0*10^-7 Nms/rad     (PAGE 8, IJRR)
    bearingVolMag = sphereVol(r)*steelSaturatedMag; %bearing volume* magnetic susceptibility
    
    % SET INITIAL CONDITIONS
    %Conventions: theta is angle of rotor
    % phi is magnetic angle
    % psi is slip angle
    %[thx, vx, thy, vy, thz, vz]
    %state0 = zeros(6,1);
    
    n=40; %number of rotors
    Rs =  generateSpacedRotors(n);
    
    %posDes = 10*[10,-10,5,-5,10,-10,8,-8]; %desired positions
    posDes = ceil(4*rand(1,n))*50-100;
    u = zeros(3,n);
    p = zeros(3,n);
    r = zeros(3,n);
    for i = 1:n  % Current position of rotor is Rs{i}*R_z(theta)*[1,0,0] (pointing to x-axis)
        u(:,i) = Rs{i}*[0;0;1]; %axis of rotation (nominally the z-axis)
        p(:,i) = Rs{i}*[0;1;0]; %perpendicular vector (nominally the point to y-axis)
        r(:,i) = Rs{i}*[1;0;0]; %rotor end (nominally the point to x-axis)
    end
    state0 = randn(n*2,1);
    
    %%%% Simulate applying a constantly rotating control law to n rotors:
    options = odeset('RelTol',1e-4,'AbsTol',1e-4);
    
    tic
    display(['Starting ', mfilename, ', ', datestr(clock)]);
    [T,Y] = ode45(@MRImultipleRotors,(0:0.05:5),state0,options);
    %[T,Y] = ode45(@MRImultipleRotors,(0:0.03:10),state0,options);
    toc
    
    figure(5)
    clf
    subplot(2,1,1)
    PHIs = zeros(numel(T),3);
    for i = 1:length(T)
        PHIs(i,:) = calcPhiVel(T(i),Y(i,:)')';
    end
    plot(T,PHIs(:,1),T,PHIs(:,2),'.-',T,PHIs(:,3),':');
    legend('X','Y','Z')
    legend('X','Y','Z','Location','east','Orientation','Horizontal');
    ylabel({'gradient';'orientation'})
    set(gcf,'papersize',[7,4])
    set(gcf,'paperposition',[0,0,7,4])
    
    save('25rotorconverge.mat', 'T','Y','n','posDes')
    
    subplot(2,1,2)
    plot(T,(Y(:,2*(1:n)-1)-repmat(posDes(1:n), size(T))).^2,'linewidth',1)
    hold on
    plot(T,sum((Y(:,2*(1:n)-1)-repmat(posDes(1:n), size(T))).^2,2),'linewidth',2)
    axis tight
    a = axis;
    axis([a(1),a(2),0,a(4)*1.05]);
    xlabel('Time (s)')
    ylabel('Error (rad^2)')
    set(gcf,'papersize',[7,5])
    set(gcf,'paperposition',[0,0,7,5])
    print -dpdf '../../pictures/pdf/PositionConverge40control.pdf'
    
    hold off
    plot(T,sum((Y(:,2*(1:n)-1)-repmat(posDes(1:n), size(T))).^2,2),'linewidth',2)
    axis tight
    a = axis;
    axis([a(1),a(2),0,a(4)*1.05]);
    xlabel('Time (s)')
    ylabel('Error (rad^2)')
    set(gcf,'papersize',[7,5])
    set(gcf,'paperposition',[0,0,7,5])
    print -dpdf '../../pictures/pdf/PositionConverge40control2.pdf'
    
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
            theta_i = Y(2*ni-1);
            vel_i = Y(2*ni);
            %calculate velocity error
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
            F = F+velErr_i*eye(3)*pV;
        end
        Phi = sign(F);
        
        
        
    end
    
    function dy = MRImultipleRotors(t,y)
        % ODE45 differential equations for multiple rotors
        PHI = calcPhiVel(t,y);
        %forces on balls
        F = bearingVolMag*maxMRIgrad*PHI;
        
        dy = zeros(size(y));    % a column vector
        for ni = 1:n  %TODO: unroll this
            theta_i = y(2*ni-1);
            vel_i = y(2*ni);
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
            dy(2*ni-1) = y(2*ni);
            %dy(2*ni  ) = 1/J*(   r1*F_i );   %works with simple model+
            %dy(2*ni  ) = 1/J*(  -b*vel_i -Tl +r1*F_i ); % works, but hits a noise floor ~50rad/s without Coulomb friction
            dy(2*ni  ) = 1/J*(  -b*vel_i -Tl +staticFriction(r1*F_i,Tfr) ); %fails with realistic model
            
            %             figure(46)
            %             clf
            %             % Plot the current position of the rotor
            %             mArrow3class.mArrow3([0 0 0], R*r(:,ni),'color','r','stemWidth',0.01,'facealpha',.5);%,'edgecolor','r')
            %             % plot the axis of the rotor
            %             mArrow3class.mArrow3([0 0 0], u(:,ni),'color','b','stemWidth',0.01,'facealpha',.5);%,'edgecolor','r')
            %             % plot the magnetic field
            %             mArrow3class.mArrow3([0 0 0], PHI,'color','k','stemWidth',0.05,'facealpha',.5);%,'edgecolor','r')
            %
            %             xlabel('x');
            %             ylabel('y');
            %             zlabel('z');
            %             % title
            %             title({sprintf('DesVel=%.1f, CurVel=%.2f',velDes(ni),vel_i);['F=',num2str(PHI')]});
            %            axis equal
            %            axis([-1,1,-1,1])
            %             % drawnow
            
        end
        
    end
    
    function updateDrawing
        drawnow
        if(MAKE_MOVIE)
            FrameCount=FrameCount+1;
            figure(f2)
            %    set(gcf,'renderer','painters')  %can  cause an error if arrows
            %    disappear.
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
        % Locate the time when we are within the desired position of our goal.
        n = numel(y)/2;
        theta = y(1:n);
        
        value = 180/pi*max(abs(posDes-theta))-45; %detect when maximum angle error<45 degrees
        
        isterminal = 1;   % stop the integration
        direction = 0;   % any direction
    end
    
end




