function DisplayRotors
    import mArrow3class.*
    n = 4;
    
    Rs =  generateSpacedRotors(n);
    
    u = zeros(3,n);
    p = zeros(3,n);
    r = zeros(3,n);
    for i = 1:n  % Current position of rotor is Rs{i}*R_z(theta)*[1,0,0] (pointing to x-axis)
        u(:,i) = Rs{i}*[0;0;1]; %axis of rotation (nominally the z-axis)
        p(:,i) = Rs{i}*[0;1;0]; %perpendicular vector (nominally the point to y-axis)
        r(:,i) = Rs{i}*[1;0;0]; %rotor end (nominally the point to x-axis)
    end
    state0 = randn(n*2,1)*2*pi;
    
    
    
    % Display Many rotors.
    figure(44)
    clf
    %hrotors = cell(1,n);
    hrotorCubes = cell(1,n);
    haxis = cell(1,n);
    htan = cell(1,n);
    range = 1:n;
    %TODO: make these all scale with n.
    rotorLength = 0.15;
    rotorwid = 0.03;
    rRadius = 0.05;
    [xsp,ysp,zsp] = sphere;
    xsp = rRadius/2*xsp;
    ysp = rRadius/2*ysp;
    zsp = rRadius/2*zsp;
    
    hSphere = cell(1,n);
    
    for ni = range
        R = rotateAboutAxisTheta(u(1,ni),u(2,ni),u(3,ni), 0);
        %Plot the current position of the rotor
        %hrotors{ni} = mArrow3class.mArrow3([0 0 0], R*r(:,ni),'color','r','stemWidth',0.01,'facealpha',.5);%,'edgecolor','r')
        %plot the axis of the rotor
        haxis{ni} = mArrow3class.mArrow3([0 0 0], R*u(:,ni),'color','k','stemWidth',0.005,'facealpha',1);%,'edgecolor','r')
        % htext{ni} = text(u(1,ni),u(2,ni),u(3,ni),num2str(ni));
        %plot the tangent direction
        htan{ni} = mArrow3class.mArrow3( R*r(:,ni), R*r(:,ni)+0.05*R*p(:,ni),'color','g','stemWidth',0.005,'facealpha',1);%,'edgecolor','r')
        
        hrotorCubes{ni} = drawCuboid ( [-.2*rotorLength,-rotorwid,0.8], [rotorLength,2*rotorwid,-0.02], 'r' );
        hold on
        pos = R*r(:,ni);
        hSphere{ni} = surf(xsp+pos(1),ysp+pos(2),zsp+pos(3),'FaceColor','b');
        set(hSphere{ni},'linestyle','none')
        
    end
%     xlabel('x');
%     ylabel('y');
%     zlabel('z');
    
    axis equal
    axis([-1,1,-1,1,-1,1])
    
    view(50,61)
    
    for theta = linspace(0,2*pi,15)
        
        for ni = range
            R = rotateAboutAxisTheta(u(1,ni),u(2,ni),u(3,ni), theta+state0(ni));
            %Plot the current position of the rotor
            %mArrow3class.updateArrow3(hrotors{ni},[0,0,0],R*r(:,ni));
            %plot the axis of the rotor
            mArrow3class.updateArrow3(haxis{ni},[0,0,0],R*u(:,ni));
            %set(htext{ni},'Position',u(:,ni));
            %plot the tangent direction
            tanArr = R*Rs{ni}*[0.60*rotorLength;0;0.8];
            mArrow3class.updateArrow3(htan{ni}, tanArr, tanArr+0.05*R*p(:,ni));
            mArrow3class.mArrow3( tanArr, tanArr+0.05*R*p(:,ni),'color','g','stemWidth',0.003,'facealpha',1);%,'edgecolor','r')
            
            for j = 1:6
                nc=R*Rs{ni}*[hrotorCubes{ni}.OrgCoords{j}.x,hrotorCubes{ni}.OrgCoords{j}.y,hrotorCubes{ni}.OrgCoords{j}.z]';
                set(hrotorCubes{ni}.patches{j}, 'xData', nc(1,:), 'yData', nc(2,:), 'zData', nc(3,:));
            end
            
            %set(hSphere{ni}, 'xdata',xsp+tanArr(1),'ydata',ysp+tanArr(2),'zdata',zsp+tanArr(3));
            
            set(hSphere{ni}, 'xdata',xsp+tanArr(1),'ydata',ysp+tanArr(2),'zdata',zsp+tanArr(3));
        end
        %    pause(0.1)
    end
    set(gca,'xticklabel',[],'yticklabel',[],'zticklabel',[])
    %set(gcf,'renderer','painters');
    set(gcf,'renderer','opengl');
    set(gcf,'papersize',[7,5])
    set(gcf,'paperposition',[0,0,7,5])
    print( '-dpdf', ['../../pictures/pdf/SimulatedRotors',num2str(n),'.pdf'])
    
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

    function R = rotateAboutAxisTheta(u_x,u_y,u_z, theta)
        % rotation matrix R for rotating theta radians about axis
        % [ux,uy,yz]
        ctheta = cos(theta);
        stheta = sin(theta);
        R = [ ctheta + u_x^2*(1-ctheta) ,       u_x*u_y*(1-ctheta) - u_z*stheta ,   u_x*u_z*(1-ctheta) + u_y*stheta
            u_y*u_x*(1-ctheta) + u_z*stheta ,   ctheta + u_y^2*(1-ctheta) ,         u_y*u_z*(1-ctheta) - u_x*stheta
            u_z*u_x*(1-ctheta) - u_y*stheta ,   u_z*u_y*(1-ctheta) + u_x*stheta ,   ctheta + u_z^2*(1-ctheta)];
    end
    
end