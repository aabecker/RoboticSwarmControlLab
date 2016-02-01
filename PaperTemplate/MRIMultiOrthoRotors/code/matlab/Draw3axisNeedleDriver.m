
function Draw3axisNeedleDriver
    
    
    
    format compact
    %have this load a file saved by Simulate3axisNeedleDriver
    
    hoopRadiusM = 0.05;% meters, the hoops guide the carriage
    hoopThickness = 0.002;
    carriageWidthHalf = 0.002; % meters
    basehingeheight = 0.008;
    gearboxLength = 0.07; %meters
    
    carriageThicknessHalf = 0.002;
    minSeparation = 116/2;   %10% separation for 0.006 mm radius beads
    baseThickness = 0.003;
    numCylinderPts = 40;
    needleLength = 0.2;
    needleRad = 0.001;
    
    hoop1color = [1,0,0];
    hoop2color = [0,0,1];
    needlecolor = [0,1,0];
    
    figure(20)
    clf
    
    
    [Xc,Yc,Zc] = cylinder(1,numCylinderPts);
    
    
    %Draw the base
    
    surf(hoopRadiusM*Xc,hoopRadiusM*Yc,baseThickness*Zc-basehingeheight-baseThickness, 'FaceColor', [0.8,0.8,0.8]);
    hold on
    
    
    %draw the hoops
    rg = floor(numCylinderPts/4+1):(floor(3*numCylinderPts/4)+1);
    
    %hoop 1
    hHoop1a = surf(hoopThickness*Zc(:,rg)-carriageWidthHalf, hoopRadiusM*Yc(:,rg),-hoopRadiusM*Xc(:,rg), 'FaceColor', hoop1color);
    hHoop1b = surf(hoopThickness*Zc(:,rg)+carriageWidthHalf, hoopRadiusM*Yc(:,rg),-hoopRadiusM*Xc(:,rg), 'FaceColor', hoop1color);
    %motor 1
    surf( basehingeheight*Xc,-Zc*gearboxLength-hoopRadiusM,basehingeheight*Yc, 'FaceColor', hoop1color)
    
    %hoop 2
    hHoop2a = surf(hoopRadiusM*Yc(:,rg), hoopThickness*Zc(:,rg)-carriageWidthHalf,-hoopRadiusM*Xc(:,rg), 'FaceColor', hoop2color);
    hHoop2b = surf(hoopRadiusM*Yc(:,rg), hoopThickness*Zc(:,rg)+carriageWidthHalf, -hoopRadiusM*Xc(:,rg), 'FaceColor', hoop2color);
    %motor 2
    surf( -Zc*gearboxLength-hoopRadiusM,basehingeheight*Xc,basehingeheight*Yc, 'FaceColor', hoop2color)
    
    
    %needle
    hNeedle = surf(needleRad*Xc,needleRad*Yc,needleLength*Zc, 'FaceColor', [0.1,0.1,0.1]);
    
    
    
    %Carriage
    
    %motor3
    hMotor3 = surf(basehingeheight*Xc+sqrt(2)*basehingeheight,basehingeheight*Yc+sqrt(2)*basehingeheight,gearboxLength*Zc+hoopRadiusM, 'FaceColor', needlecolor);
    
    
    
    %axis tight
    axis equal
    
    for i = 1:20
        rotate(hHoop1a,[0,1,0],2,[0,0,0])
        rotate(hHoop1b,[0,1,0],2,[0,0,0])
        
        rotate(hHoop2a,[1,0,0],2,[0,0,0])
        rotate(hHoop2b,[1,0,0],2,[0,0,0])
        
        
         rotate(hNeedle,[1,0,0],2,[0,0,0])
        rotate(hNeedle,[0,1,0],2,[0,0,0])
        
         rotate(hMotor3,[1,0,0],2,[0,0,0])
        rotate(hMotor3,[0,1,0],2,[0,0,0])
        
        pause(0.1)
        
        
    end
end


function rotateObject( hObj, axis, angleDeg)
    
    set(hObj, 'Xdata', hObj.origXData);
    set(hObj, 'Ydata', hObj.origYData);
    set(hObj, 'Zdata', hObj.origZData);
    
    rotate(hObj,axis,angleDeg,[0,0,0]);
    
end


function [x,y,z] = solidCylinder(varargin)  %TODO: convert this to mathematica style.
    
    %// Basic checks
    assert(nargin >= 1, 'Not enough input arguments.');
    assert(nargin <= 3, 'Too many input arguments.');
    assert(nargout <= 3, 'Too many output arguments.');
    
    %// Parse input
    N  = 20;
    Ax = [];
    switch nargin
        case 1 %// R
            R  = varargin{1};
        case 2  %// Ax, R  or  R, N
            if ishandle(varargin{1})
                Ax = varargin{1};
                R  = varargin{2};
            else
                R  = varargin{1};
                N  = varargin{2};
            end
            
        case 3 %// Ax, R, N
            Ax = varargin{1};
            R  = varargin{2};
            N  = varargin{3};
    end
    
    %// Check input arguments
    if ~isempty(Ax)
        assert(ishandle(Ax) && strcmp(get(Ax, 'type'), 'axes'),...
            'Argument ''Ax'' must be a valid axis handle.');
    else
        Ax = gca;
    end
    
    assert(isnumeric(R) && isvector(R) && all(isfinite(R)) && all(imag(R)==0) && all(R>0),...
        'Argument ''R'' must be a vector containing finite, positive, real values.');
    assert(isnumeric(N) && isscalar(N) && isfinite(N) && imag(N)==0 && N>0 && round(N)==N,...
        'Argument ''N'' must be a finite, postive, real, scalar integer.');
    
    %// Compute cylinder coords (mostly borrowed from builtin 'cylinder')
    theta         = 2*pi*(0:N)/N;
    sintheta      = sin(theta);
    sintheta(N+1) = 0;
    
    M = length(R);
    if M==1
        R = [R;R]; M = 2; end
    
    x = R(:) * cos(theta);
    y = R(:) * sintheta;
    z = (0:M-1).'/(M-1) * ones(1,N+1);  %'
    
    if nargout == 0
        oldNextPlot = get(Ax, 'NextPlot');
        set(Ax, 'NextPlot', 'add');
        
        %// The side of the cylinder
        surf(x,y,z, 'parent',Ax);
        %// The bottom
        patch(x(1,:)  , y(1,:)  , z(1,:)  , z(1,:)  );
        %// The top
        patch(x(end,:), y(end,:), z(end,:), z(end,:));
        
        set(Ax, 'NextPlot', oldNextPlot);
    end
    
end
