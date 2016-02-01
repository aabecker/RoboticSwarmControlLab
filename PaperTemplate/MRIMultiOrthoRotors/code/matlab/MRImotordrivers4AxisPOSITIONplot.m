function MRImotordrivers4AxisPOSITIONplot()
% plots 4 rotors being controlled simultaneously by MRI gradient field
% doing position control
% Works lovely.

%  Author: Aaron Becker
%  Date: 10/18/2013
%close all
format compact
set(0,'DefaultAxesFontSize',14)
set(0,'defaultaxesfontsize',14);
%set(0,'DefaultTextInterpreter', 'latex')


MAKE_MOVIE = true;
%  Seed the generator
rng(10);
% CONSTANTS, section IV, IJRR article
steelSaturatedMag = 1.36*10^6;      % A/m  http://hyperphysics.phy-astr.gsu.edu/hbase/tables/magprop.html
maxMRIgrad = 0.040;                 %T/m  (page 4, IJRR)
steelBearingDiamInmm = 6;           %= [1,1.5,2,2.5,3,3.5,4,4.5,6,6.5,7,8,9,10,12,13,14,15,16,18,24]; % mm, bearing sizes available from McMaster
r = steelBearingDiamInmm/2*0.001;   % radius in m
Tfr =  7.0*10^-5;                   % 7*10^-5Nm    =  kg*m/(s^2) (friction PAGE 9, IJRR)
Tl = 0.0*10^-5;                              % load torque
m = sphereVol(r)*8530;              %m^3* kg / (m^3), mass of bearing
r1 = 0.018;                         %m, radius rotor arm  with steel bearing.
J = 2*m*r1^2;                       % total inertia
b = 1.0*10^-7;                     %1.0*10^-7 Nms/rad     (PAGE 9, IJRR)
bearingVolMag = sphereVol(r)*steelSaturatedMag; %bearing volume* magnetic susceptibility

% SET INITIAL CONDITIONS
%Conventions: theta is angle of rotor
% phi is magnetic angle
% psi is slip angle
%[thx, vx, thy, vy, thz, vz]
%state0 = zeros(6,1);

n=4; %number of rotors
Rs = cell(n,1);
Rs{1} = eye(3);  %rotate about the z-axis
Rs{2} = RxTheta(-pi/2); %rotate about the y-axis
Rs{3} = RyTheta(pi/2)*RzTheta(pi/2); %rotate about the x-axis
Rs{4} = RyTheta(pi/4);%rotate about the xz-axis
 Rs{5} = RxTheta(-pi/4);%rotate about the zy-axis
 Rs{6} = RzTheta(-pi/4)*RxTheta(-pi/2); %rotate about the xy-axis
 Rs{7} = RzTheta(-pi/4)*RxTheta(-pi/4); %center it
 Rs{8} = RzTheta(-pi/4)*RxTheta(-3*pi/4); %rotate about the xy-axis
%Rs{1} = RxTheta(-pi/4);%rotate about the zy-axis
%Rs{2} = RzTheta(-pi/4)*RxTheta(-pi/2); %rotate about the xy-axis

posDes = 10*[10,-10,7.5,-5,10,-10,8,-8]; %desired positions
%posDes = ceil(4*rand(1,n))*50-100;
u = zeros(3,n);
p = zeros(3,n);
r = zeros(3,n);
for i = 1:n  % Current position of rotor is Rs{i}*R_z(theta)*[1,0,0] (pointing to x-axis)
    u(:,i) = Rs{i}*[0;0;1]; %axis of rotation (nominally the z-axis)
    p(:,i) = Rs{i}*[0;1;0]; %perpendicular vector (nominally the point to y-axis)
    r(:,i) = Rs{i}*[1;0;0]; %rotor end (nominally the point to x-axis)
end
state0 = randn(n*2,1);


%%%% Simulate applying a constantly rotating control law to 3 rotors:
options = odeset('RelTol',1e-3,'AbsTol',1e-3);
tic
display(['Starting ',mfilename, ', ', datestr(clock)]);
[T,Y] = ode45(@MRImultipleRotors,[0,8],state0,options);
%[T,Y] = ode45(@MRImultipleRotors,(0:0.03:10),state0,options);
toc

figure(4)
clf

for ci = 1:n
    subplot(n/2,2,ci)
    %plot(T,Y(:,2*ci-1),'-',T,Y(:,2*ci),'-',T,posDes(ci)*ones(size(T)),'--')
    plot(T,Y(:,2*ci-1),'-',T,posDes(ci)*ones(size(T)),'--','linewidth',1.5)
    if ci == 1
        legend('rad','goal','Location','SouthEast')
        %legend('rad', 'rad/s','goal','Location','NorthWest')
    end
    
    if ci == 1 || ci == 3
        ylabel('rad')
    end
    if ci == 3 || ci == 4
        xlabel('time [s]')
    end
    text(4,(max(Y(:,2*ci-1))+min(Y(:,2*ci-1)))/2, ['Axis \theta_',num2str(ci),''],'fontsize',14)
    
    axis tight
end
set(gcf,'position',[195    49   560   948])

set(gcf,'papersize',[7,4])
set(gcf,'paperposition',[0,0,7,4])
%print -dpdf '../../pictures/pdf/PositionConverge4state.pdf'

colors = [0.2472, 0.24, 0.6;
        0.6, 0.24, 0.442893;
0.6, 0.547014, 0.24;
0.24, 0.6, 0.33692;
0.24, 0.353173, 0.6;
0.6, 0.24, 0.563266;
0.6, 0.426641, 0.24;
0.263452, 0.6, 0.24;
0.24, 0.473545, 0.6;
0.516361, 0.24, 0.6];


figure(6)
clf

for i = 1:n
    plot(T,posDes(i)*ones(size(T)),'linestyle', '--','Color',colors(i,:),'linewidth',1.5);
    hold on 
    plot(T,Y(:,2*i-1),'linestyle', '-','Color',colors(i,:),'linewidth',1.5);
    text(4,posDes(i)-12*sign(posDes(i)), ['Axis \theta_',num2str(i),''],'fontsize',14)
end
    


        legend('goal','\theta','Location','East')
        ylabel('Rotor Angle [rad]')
        xlabel('time [s]')

  axis tight
  a=  axis;
axis([a(1),a(2),a(3)*1.05,a(4)*1.05]);
set(gcf,'position',[195    49   560   948])
set(gcf,'papersize',[7,4])
set(gcf,'paperposition',[0,0,7,4])
print -dpdf '../../pictures/pdf/PositionConverge4stateNoload.pdf'


figure(5)
clf
subplot(3,1,1)
PHIs = zeros(numel(T),3);
for i = 1:length(T)
    PHIs(i,:) = calcPhiVel(T(i),Y(i,:)')';
end

ie =find(T>=3,1,'first');
rg = 1:ie-1;

subplot(2,1,1)
%plot(T(rg),PHIs(rg,1),T(rg),PHIs(rg,2),'.-',T(rg),PHIs(rg,3),':','linewidth',1.5);
hC =stairs(T(rg),[PHIs(rg,1),PHIs(rg,2),PHIs(rg,3)]);
set(hC,'linewidth',1.5)
set(hC(2),'LineStyle','-.')
set(hC(3),'LineStyle',':')
legend('X','Y','Z','Location','East','Orientation','Horizontal');

ylabel({'gradient';'orientation'})


subplot(2,1,2)
for i = 1:n
plot(T(rg),(Y(rg,2*i-1)-repmat(posDes(i), size(T(rg)))).^2,'linewidth',1,'Color',colors(i,:))
hold on
end
plot(T(rg),sum((Y(rg,2*(1:n)-1)-repmat(posDes(1:n), size(T(rg)))).^2,2),'linewidth',2)
xlabel('Time [s]')
ylabel('Error [rad^2]')


set(gcf,'papersize',[7,4])
set(gcf,'paperposition',[0,0,7,4])
print -dpdf '../../pictures/pdf/PositionConverge4controlNoload.pdf'

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% MAKE A MOVIE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if MAKE_MOVIE
    import tacho.*
    import mArrow3class.*
    f2 =figure(2);
    clf
    range = 1:n;
    FrameCount = 1;
    MOVIE_NAME = 'LyapunovMultiAxisPosition';
    G.fig = figure(2);
    set(G.fig,'Units','normalized','outerposition',[0 0 1 1],'NumberTitle','off');%'MenuBar','none',
    writerObj = VideoWriter(MOVIE_NAME,'MPEG-4');%http://www.mathworks.com/help/matlab/ref/videowriterclass.html
    set(writerObj,'Quality',100)
    %set(writerObj, 'CompressionRatio', 2);
    open(writerObj);
    
    
    OrientRange = ceil(max( max(abs(Y(:,2*(1:n)-1))))/5)*5; %rad
    %SETUP PLOTS
    hTach = cell(n,1);
    for ni = range
        
        hTach{ni} = subplot(4,ceil(n/4)+3,mod(ni-1,2)+1 + floor((ni-1)/2)*5);
        %initTacho(axeshandle,minvalue,maxvalue,steps,substeps,textsteps,unit,needlecolor)
        tacho.initTacho(hTach{ni},-OrientRange,OrientRange,12,24,3,{'x (rad)';num2str(ni)},'r');
        tacho.addGoalTacho(hTach{ni},posDes(ni),[0.5,0,0])
    end
   % % % Display the different rotors.

    hrotorCubes = cell(1,n);
    haxis = cell(1,n);
    htan = cell(1,n);
    htext = cell(1,n);
    subplot(4,ceil(n/4)+3,[3:5,8:10,13:15,18:20])
    rotorLength = 0.3;
    for ni = range
        R = rotateAboutAxisTheta(u(1,ni),u(2,ni),u(3,ni), 0);
        %plot the axis of the rotor
        haxis{ni} = mArrow3class.mArrow3([0 0 0], R*u(:,ni),'color','b','stemWidth',0.01,'facealpha',.5);%,'edgecolor','r')
        htext{ni} = text(u(1,ni),u(2,ni),u(3,ni),num2str(ni));
        %plot the tangent direction
        htan{ni} = mArrow3class.mArrow3( R*r(:,ni), R*r(:,ni)+0.2*R*p(:,ni),'color','g','stemWidth',0.005,'facealpha',.5);%,'edgecolor','r')
        %Plot the current position of the rotor
        hrotorCubes{ni} = drawCuboid ( [-.1*rotorLength,-0.05,0.8], [rotorLength,0.1,-0.02], 'r' );
    end
    
    
    arrX = [0,0,    0.7,0.7,  1,   0.7, 0.7, 0,   0];
    arrY = [0,0.1,  0.1,0.2,  0,  -0.2,-0.1,-0.1, 0];
    arrZ = ones(size(arrX));
    grey = [0.6,0.6,0.6];
    mX = patch(arrZ,arrY,arrX,grey);
    mY = patch(arrY,arrZ,arrX,grey);
    mZ = patch(arrX,arrY,-arrZ,grey);

    
    xlabel('x');
    ylabel('y');
    zlabel('z');
    %title
    htitle = title(sprintf('Orthogonal Rotor Position Control, t=%2.1fs',0));
    
    axis equal
    axis([-1,1,-1,1,-1,1])
    grid on

    set(gca,'xTick',-1:0.5:1,'yTick',-1:0.5:1,'zTick',-1:0.5:1)
    axis([-1.0001,1.001,-1.0001,1.0001,-1.0001,1.0001])
    set(gca,'gridlinestyle','-')

    for i = 1:length(T)
        phi = calcPhiVel(T(i),Y(i,:)');
        for ni = range
            tacho.updateTacho(hTach{ni},Y(i,2*(ni)-1));
        end
        
        for ni = range
            %Y(i,2*(1:ni)-1)
            R = rotateAboutAxisTheta(u(1,ni),u(2,ni),u(3,ni), Y(i,2*(ni)-1));
            %plot the axis of the rotor
            mArrow3class.updateArrow3(haxis{ni},[0,0,0],R*u(:,ni));
            set(htext{ni},'Position',u(:,ni));
            %plot the tangent direction
            tanArr = R*Rs{ni}*[0.9*rotorLength;0;0.8];
            mArrow3class.updateArrow3(htan{ni}, tanArr, tanArr+0.2*R*p(:,ni));
            mArrow3class.mArrow3( tanArr, tanArr+0.15*R*p(:,ni),'color','g','stemWidth',0.003,'facealpha',.5);%,'edgecolor','r')
            for j = 1:6
                nc=R*Rs{ni}*[hrotorCubes{ni}.OrgCoords{j}.x,hrotorCubes{ni}.OrgCoords{j}.y,hrotorCubes{ni}.OrgCoords{j}.z]';
                set(hrotorCubes{ni}.patches{j}, 'xData', nc(1,:), 'yData', nc(2,:), 'zData', nc(3,:));
            end
        end
        
        %calculate the magnetic angle in each direction.
        ax = atan2(phi(3),phi(2));  mx =sqrt( phi(3)^2+phi(2)^2);
        ay = atan2(phi(1),phi(3));  my =sqrt( phi(1)^2+phi(3)^2);
        az = atan2(phi(2),phi(1));  mz =sqrt( phi(2)^2+phi(1)^2);
        set(mX, 'YData', mx*(cos(ax)*arrX-sin(ax)*arrY),'ZData',  mx*(sin(ax)*arrX+cos(ax)*arrY), 'XData',arrZ);
        set(mY, 'ZData', my*(cos(ay)*arrX-sin(ay)*arrY), 'XData', my*(sin(ay)*arrX+cos(ay)*arrY),'YData', arrZ);
        set(mZ, 'XData', mz*(cos(az)*arrX-sin(az)*arrY), 'YData', mz*(sin(az)*arrX+cos(az)*arrY),'ZData', -arrZ);
        
        set(htitle,'string',sprintf('Orthogonal Rotor Position Control, t=%2.1fs',T(i)));
        %drawnow
        updateDrawing
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%% END MAKE A MOVIE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

    function v = sphereVol(r)
        v = 4/3*pi*r.^3;
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
            gain =  1;  %this number is almost meaningless.  just needs to be positive
            posErr_i = gain*(posDes(ni) - theta_i); %^2*sign(posDes(ni) - theta_i);  
            velErr_i = posErr_i- vel_i;
            %compute perpendicular vector
            R = rotateAboutAxisTheta(u(1,ni),u(2,ni),u(3,ni), theta_i);
            pV = R* p(:,ni);
            %project force onto perpendicular vector
            %F = F+posErr_i*eye(3)*pV;
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
            dy(2*ni  ) = 1/J*(  -b*vel_i  +staticFriction(r1*F_i-Tl,Tfr) ); %fails with realistic model

        end
        
    end
    function h = drawCuboid ( origin, size, color )
        %draws cuboid of color color centered at the origin
        x=([0 1 1 0 0 0;1 1 0 0 1 1;1 1 0 0 1 1;0 1 1 0 0 0]-0.5)*size(1)+origin(1)+size(1)/2;
        y=([0 0 1 1 0 0;0 1 1 0 0 0;0 1 1 0 1 1;0 0 1 1 1 1]-0.5)*size(2)+origin(2)+size(2)/2;
        z=([0 0 0 0 0 1;0 0 0 0 0 1;1 1 1 1 0 1;1 1 1 1 0 1]-0.5)*size(3)+origin(3)+size(3)/2;
        h.patches = cell(6,1);
        h.OrgCoords = cell(6,1);
        for mj=1:6
            h.patches{mj} =patch(x(:,mj),y(:,mj),z(:,mj),color);
            set(h.patches{mj},'edgecolor','k')
            h.OrgCoords{mj}.x = x(:,mj);
            h.OrgCoords{mj}.y = y(:,mj);
            h.OrgCoords{mj}.z = z(:,mj);
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

end




