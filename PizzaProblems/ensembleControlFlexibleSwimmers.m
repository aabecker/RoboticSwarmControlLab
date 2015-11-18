function ensembleControlFlexibleSwimmers
% Simulates control of flexible swimmers
% 
%TODO: 
%

SHOW_MOVIE = false; %animation showing the magnetic field
global  freq vel a running goal endBound endTime timeFlag N x y th phase
running = 1;  %if you close the figure with the movie, this cleanly shuts down the program
set(0,'defaultaxesfontsize',14);
set(0,'defaulttextfontsize',14);
format compact

%%%%%%%%%%%%%%%%%% global values %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
N = 4; %number of robots
a = linspace(0,1/(N+1)*2*Pi,N/(N+1)*2*Pi)'; % a values, should be between 3 and 10
x = 20*ones(N,1);
y = linspace(-20,20,N)';  %spaces cells vertically
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

timeFlag = 0;
tF = 100;  %final time, seconds

%initial conditions for robots (x,y)
options = odeset('RelTol',1e-4,'AbsTol',1e-4 );
[T,Y] = ode45(@simFlexibleSwimmers,[0 tF],[x;y],options);

%%%%%%%%%%%%%%%%%%%%%%% PLOTTING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%draw path, cells
figure
clf
c = ['r','g','m','y','k','c','b','r','g','m','y','k','c','b'];
for i = 1:N
    plot(Y(1,i),Y(1,i+N),'x','color',c(i))
    hold on
    plot(Y(:,i),Y(:,i+N),'-','color',c(i))
    text(Y(1,i),Y(1,i+N)+0.2,['a = ',num2str(a(i)),'; v = ',num2str(vel(i)),' \mum/s'])
end
axis equal
title({[num2str(N),' cells simulated for ',num2str(endTime),' s','; freq = ',num2str(freq),'rad/s'];});%['\theta_M = ',num2str(Mag),'sin(',num2str(freq),'t)']});
xlabel('mm')
ylabel('mm')
for i = 1:N
    drawCell(Y(end,i),Y(end,i+N),Y(end,i+2*N),a(i),0.05);
end


%%%%%%%%%%%%%%%%%%%%%% plot ending bound %%%%%%%%%%%%%%%%%%%%%%%%%
thetaBound = linspace(0,2*pi,1000);
xBound = endBound*cos(thetaBound);
yBound = endBound*sin(thetaBound);
hold on
plot(xBound,yBound,'r*');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ax = axis;
endTime = tF;

figure %plot sum squared error 
T = linspace(0,endTime,length(Y(:,1)))';
r = vel./(freq); % radius of circle the robot is turning around
  %(xc,yc) is the center of the circle the robot is following
% th = Y(:,1+2*N:3*N);
% xc = Y(:,1+0*N:1*N)  -r*cos(th-pi/2);
% yc = Y(:,1+1*N:2*N)  -r*sin(th-pi/2);
xc = Y(:,1+0*N:1*N);
yc = Y(:,1+1*N:2*N);
%sum((Y(:,1:nRob).^2 + Y(:,nRob+1:2*nRob).^2).^.5,2)./nRob;
for i = 1:N
    plot( T, ((xc(:,i)-goal(1)).^2+(yc(:,i)-goal(1)).^2).^1,'linewidth',1,'color',c(i));
    hold on
end

plot( T, sum(((xc-goal(1)).^2+(yc-goal(1)).^2).^1,2),'linewidth',2,'color',c(N+1))
xlabel('time')
ylabel('sum squared error')
title('Lyapunov Function')


if ~SHOW_MOVIE
    return
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%% MAKE A MOVIE! %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Setup figures %%%%%%%%%
figure;
set(gcf,'CloseRequestFcn',@F2Close);

clf
hpath = zeros(N,1);
hCell  = cell(N,1);
c = ['r','g','m','y','k','c','b'];
for i = 1:N
    %start pos
    plot(Y(1,i),Y(1,i+N),'x','color', c(i))
    hold on
    hpath(i) = plot(Y(1,i),Y(1,i+N),'-','color', c(i),'linewidth',2);
    ht =text(Y(1,i),Y(1,i+N)+1,['a = ',num2str(a(i))]);
    uistack(ht,'top')
end
for i = 1:N
    %draw cells
    hCell{i} = drawCell(Y(1,i),Y(1,i+N),Y(1,i+2*N),a(i),0.05);
end
hTitle = title({[num2str(N),' cells simulated for ',num2str(tF),' s']});%;['\theta_M = ',num2str(Mag),'sin(',num2str(freq),'t), T = ',num2str(T(1)),' s']});


Magsc = 1;
MagRad = max(ax(2)-ax(1),ax(4)-ax(3))/2;
MagOrigin = [ (ax(2)+ax(1))/2, (ax(4)+ax(3))/2];
axis equal
ax(1) = ax(1)-2*Magsc;
ax(2) = ax(2)+2*Magsc;
ax(3) = ax(3)-2*Magsc;
ax(4) = ax(4)+2*Magsc;
axis(ax);
hMagLine = zeros(6*N,1);
hMagLineAr = zeros(6*N,1);
magcolor = [.5,.5,1];
for i = 1:numel(hMagLine)
    %draw lines
    hMagLine(i) = plot( MagOrigin(1)+[-3*MagRad,3*MagRad],...
        MagOrigin(2)+(i-numel(hMagLine)/2)*3*MagRad/numel(hMagLine)*[1,1],'Color',magcolor);
    uistack(hMagLine(i),'bottom')
    hMagLineAr(i) = patch( MagOrigin(1)+[0,0,Magsc],...
        MagOrigin(2)+(i-numel(hMagLine)/2)*3*MagRad/numel(hMagLine)+[-Magsc/4,Magsc/4,0],magcolor);
    set(hMagLineAr(i),'edgecolor',magcolor);
    uistack(hMagLineAr(i),'bottom')
end
%%%% show movie frames %%%%%%%%%%%%%%%%%%%
for j = 1:numel(T)
    if ~running
        break
    end
    set(hTitle,'string',{[num2str(N),' cells simulated for ',num2str(tF),' s']});%['\theta_M = ',num2str(Mag),'sin(',num2str(freq),'t), T = ',num2str(T(j), '% 5.0f'),' s']});
    
    for i = 1:N
        set(hpath(i),'Xdata',Y(1:j,i),'Ydata',Y(1:j,i+N));
        updateCell(hCell{i},Y(j,i),Y(j,i+N),Y(j,i+2*N));
    end
    %thetaM = Mag*sin(freq*T(j));
    thetaM = (freq*T(j));
    %updateMagnet(hMag, thetaM);
    for i = 1:numel(hMagLine)
        %draw lines
        xmag =  [-3*MagRad,3*MagRad];
        ymag = (i-numel(hMagLine)/2)*3*MagRad/numel(hMagLine)*[1,1];
        set(hMagLine(i),'Xdata',MagOrigin(1)+xmag*cos(thetaM)-ymag*sin(thetaM), 'Ydata',MagOrigin(2)+xmag*sin(thetaM)+ymag*cos(thetaM));
        
        xar=20+[0,0,Magsc];
        yar=(i-numel(hMagLine)/2)*3*MagRad/numel(hMagLine)+[-Magsc/4,Magsc/4,0];
        set(hMagLineAr(i),'Xdata',MagOrigin(1)+xar*cos(thetaM)-yar*sin(thetaM), 'Ydata',MagOrigin(2)+xar*sin(thetaM)+yar*cos(thetaM));
        
    end
    
    axis(ax);
    drawnow
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%   END MOVIE!  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end


function F2Close(~,~)
global running
running = 0;
delete(gcf);
end

function dq = simFlexibleSwimmers(t,q)
% simulates a differential equation model for flexible swimmers
% q = [x_coordinates; y_coordinates;
global  freq vel a goal endBound timeFlag endTime
M = 1;  %magnetic field, 1= on, 0 = off
goal = [0,0];  %desired position of robots

dq = zeros(size(q));    % a column vector for the change in state
N = (numel(q)-1)/3;  %number of robots

xc = q(0*N+1:1*N);
yc = q(1*N+1:2*N);

%goal = [mean(xc),mean(yc)]; %to collect all robots together (clumping/rendezvous)
xerr  = goal(1)-xc; %error between goal position and current circle center
yerr  = goal(2)-yc;

F = -sum( vel.*xerr.*cos(th)+vel.*yerr.*sin(th));  %gradient of the distance
 
% if the cells is in some bound, stop
if sum((q(1:N).^2+q(N+1:2*N).^2) < endBound^2) == N
    if timeFlag == 0
       endTime = t;
        timeFlag = timeFlag+1;
    end
    return
end
 
 
dq(1:N)       = vel.*cos(th);
dq(N+1:2*N)   = vel.*sin(th);

end