function MultiAxisCalcTorque(n,NUM_TESTS,savefilename)
% what is the average torque we can produce with multiple rotors all
% actuated by the same magnetic field?
%
% DONE:  Q.) Is this the correct way?  What happens if the errors on each axis are randomly negative or positive?
% Shouldn't I sample with different values of Error?
% A.)  this doesn't seem to matter -- I generated random +/- desired turns
% for each axis, and I get the sameuseful torque outputs (since each rotor
% has equal probability of being at phi or phi+pi).
%
%  One way to compute this is
%  T_{sum} = \sum_{i=1}^n (  Torque on rotor i  at a given configuration)
% the average torque is then 1/(2*\pi) \Int_0^{2\pi} T_{sum}  d\theta_1    *use the terminology at http://en.wikipedia.org/wiki/Multiple_integral
% \frac{1}{2*\pi} \int \cdots \int_\mathbf{D}\;T_{sum} \;d\theta_1 \!\cdots d\theta_n
%  Author: Aaron Becker
%  Date: 10/18/2013
%close all

%
%  TODO:
%0: generate rotor spacing automatically.
%1.  plot the average totaltorque & variance as a function of n
%2. plot the average individual torque & variance as a function of n
%3. plot compare the results of not using ideal rotor spacing for n = 3 andn = 4;
%4. save the data to a file. (DONE!)

format compact
%clc
set(0,'DefaultAxesFontSize',14)


%  Seed the generator
%rng(10);
% CONSTANTS, section IV, IJRR article
steelSaturatedMag = 1.36*10^6;      % A/m  http://hyperphysics.phy-astr.gsu.edu/hbase/tables/magprop.html
maxMRIgrad = 0.040;                 %T/m  (page 4, IJRR)
steelBearingDiamInmm = 6;           %= [1,1.5,2,2.5,3,3.5,4,4.5,6,6.5,7,8,9,10,12,13,14,15,16,18,24]; % mm, bearing sizes available from McMaster
r = steelBearingDiamInmm/2*0.001;   % radius in m
bearingVolMag = sphereVol(r)*steelSaturatedMag; %bearing volume* magnetic susceptibility

if nargin<3
    savefilename = 'MultiAxisCalcTorqueData.mat';
end

if nargin < 1
    n=2; %number of rotors
end

savefilename = ['MultiAxisCalc/',savefilename];

% Rs = cell(n,1);
% Rs{1} = eye(3);  %rotate about the z-axis   COULD DO BETTER if oriented to use 3 mag gradients simultaneously.
% if n>1
% Rs{2} = RxTheta(-pi/2); %rotate about the y-axis
% end
% if n>2
% Rs{3} = RyTheta(pi/2)*RzTheta(pi/2); %rotate about the x-axis
% end
% if n > 3
 Rs =  generateSpacedRotors(n);
% end

%Rs =  generateRandRotors(n);

% Rs{4} = RyTheta(pi/4);%rotate about the xz-axis
% Rs{5} = RxTheta(-pi/4);%rotate about the zy-axis
% Rs{6} = RzTheta(-pi/4)*RxTheta(-pi/2); %rotate about the xy-axis
% Rs{7} = RzTheta(-pi/4)*RxTheta(-pi/4); %center it
% Rs{8} = RzTheta(-pi/4)*RxTheta(-3*pi/4); %rotate about the xy-axis
% %Rs{1} = RxTheta(-pi/4);%rotate about the zy-axis
% %Rs{2} = RzTheta(-pi/4)*RxTheta(-pi/2); %rotate about the xy-axis

%  The spacing I generated by hand above was not uniformly distributed,
%  which means that we could generate stronger torques, but with less
%  independence.
% also, you can get BIGGER torques for n=1 by being able to use 3 mag
% fields at once...  Same story for n=2, I think.
minA = distanceBetweenRotationMatrices(Rs);



    function Rs =  generateRandRotors(n)
        % generate n poorly-spaced rotors.
        Rs = cell(n,1);
        %x = rand(1,2*n);  %first n are lat (elevation), next n are longitude (azimuth)
        x = zeros(1,2*n);   %all point up
        r = x(1:n);
        ind = r>pi;
        while sum(ind)
            r(ind) = r(ind)-pi;
            ind = r>pi;
        end
        ind = r<0;
        while sum(ind)
            r(ind) = r(ind)+pi;
            ind = r<0;
        end
        t = x(n+1:2*n);
        [X, Y, Z] = sph2cart(t, r, 1);
        v = [X', Y', Z'];
        
        nomAxisOfRotation = [0;0;1];
        eps = 0.001;
        for ni = 1:n
            theta = acos(dot( nomAxisOfRotation,v(ni,:)));
            if theta < eps
                Rs{ni} = eye(3);
            else
                if pi-theta < eps
                    x = [1;0;0];
                else
                    x = cross(nomAxisOfRotation,v(ni,:))/norm(cross(nomAxisOfRotation,v(ni,:)));
                end
                Rs{ni} = rotateAboutAxisTheta(x(1),x(2),x(3), theta);
            end
        end
    end

    function minA = distanceBetweenRotationMatrices(Rs)
        % what is the minimum angle between any two rotation matrices in
        % Rs?
        n = numel(Rs);
        angs = zeros(n);
        minA = pi;
        for ni = 1:n
            for mi = ni+1:n
                angs(ni,mi) = minAngle2Rot(Rs{ni},Rs{mi});
                if angs(ni,mi) < minA
                    minA = angs(ni,mi);
                end
            end
        end
        minA = minA*180/pi;
        display(minA)
        display(angs)
    end

    function phi = minAngle2Rot(Ra,Rb)
        %minimum angle between two rotation matrices, a type of metric
        phi = acos(  ( trace( Ra'*Rb)-1 )/2 );
    end


posDes = 10*[10,-10,5,-5,10,-10,8,-8]; %desired positions

u = zeros(3,n);
p = zeros(3,n);
r = zeros(3,n);
for i = 1:n  % Current position of rotor is Rs{i}*R_z(theta)*[1,0,0] (pointing to x-axis)
    u(:,i) = Rs{i}*[0;0;1]; %axis of rotation (nominally the z-axis)
    p(:,i) = Rs{i}*[0;1;0]; %perpendicular vector (nominally the point to y-axis)
    r(:,i) = Rs{i}*[1;0;0]; %rotor end (nominally the point to x-axis)
end

if nargin<2
    NUM_TESTS = 100000;
end
% generate 1000 random vectors of rotor positions  (0, 2*pi)
vectors = rand(NUM_TESTS,n)*2*pi;
torques = zeros(NUM_TESTS,n);
% compute the torque under our control law  (set velocity error to 1)
tic
for i = 1:NUM_TESTS
    torques(i,:) = torqueMRImultipleRotors(vectors(i,:));
end
toc
% record the mean, max, min, and variance, and number of tests to a data
% file
if exist(savefilename, 'file') == 2
    load(savefilename,'NUM_TESTS_Total', 'TORQUES_MEAN','TORQUES_MIN','TORQUES_MAX','TORQUES_VAR','TORQUES_ROWSUM_MEAN','TORQUES_ROWSUM_VAR','TORQUES_ROWSUM_AVE' )
end

%TODO: clean these so they update with respect to data we have already.
TORQUES_MEAN{n} = mean(torques);
TORQUES_MAX{n} = max(torques);
TORQUES_MIN{n} =min(torques);
TORQUES_VAR{n} = var(torques);
TORQUES_ROWSUM = sum(torques,2);

TORQUES_ROWSUM_MEAN(n) = mean(TORQUES_ROWSUM);
TORQUES_ROWSUM_AVE(n) = TORQUES_ROWSUM_MEAN(n)/n;
TORQUES_ROWSUM_VAR(n) = var(TORQUES_ROWSUM  );
NUM_TESTS_Total(n) = NUM_TESTS;% NUM_TESTS_Total(n) + NUM_TESTS;

save(savefilename,'NUM_TESTS_Total', 'TORQUES_MEAN','TORQUES_MIN','TORQUES_MAX','TORQUES_VAR','TORQUES_ROWSUM_MEAN','TORQUES_ROWSUM_VAR','TORQUES_ROWSUM_AVE' )

display({[savefilename,':',num2str(n)];['NUM_TESTS_Total=',num2str(NUM_TESTS_Total(n)), ', TORQUES_MEAN=',num2str(TORQUES_MEAN{n}), ', TORQUES_MIN=',num2str(TORQUES_MIN{n}),...
    ','];['TORQUES_MAX=',num2str(TORQUES_MAX{n}), ', TORQUES_VAR=',num2str(TORQUES_VAR{n}), ', TORQUES_ROWSUM_MEAN=',num2str(TORQUES_ROWSUM_MEAN(n)),...
    ','];['TORQUES_ROWSUM_VAR=',num2str(TORQUES_ROWSUM_VAR(n)), ', TORQUES_ROWSUM_AVE=',num2str(TORQUES_ROWSUM_AVE(n))]});


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

    function Phi = calcPhiVel(Y)%,ERRORS)
        % minimizes the candidate Lyapunov
        % function:  V() = sum ( 1/2 *(vel_{des} - vel_{act})^2 )
        %            Vdot() = sum( (vel_{des} - vel_{act})*Projection Of [Fx;Fy;Fz]
        F = zeros(3,1);
        for ni = 1:n
            theta_i = Y;
            %calculate velocity error
            posErr_i = 1;
            %compute perpendicular vector
            R = rotateAboutAxisTheta(u(1,ni),u(2,ni),u(3,ni), theta_i(ni));
            pV = R* p(:,ni);
            %project force onto perpendicular vector
            F = F+posErr_i*eye(3)*pV;
            %F = F+ERRORS(ni)*eye(3)*pV;
        end
        Phi = sign(F);
        
    end

    function T = torqueMRImultipleRotors(y)
        % ODE45 differential equations for multiple rotors
        %ERRORS = (-1).^(randi(2,[n,1])-1);
        PHI = calcPhiVel(y);%,ERRORS);
        %forces on balls  (assume F is between -1 and 1)
        F = PHI;
        T = zeros(size(y));    % a column vector
        
        
        for ni = 1:n  %TODO: unroll this
            theta_i = y(ni);
            %NOMINAL rotor rotates about the axis [0,0,1], for theta = 0 ferrite at [1,0,0];
            % ith rotor rotates about axis specified by unit vector u_i, to
            % angle theta_i
            %Step 1: rotate theta_i around axis
            R = rotateAboutAxisTheta(u(1,ni),u(2,ni),u(3,ni), theta_i);
            %compute perpendicular vector
            pV = R*p(:,ni);
            %project force onto perpendicular vector  (assume rotor is of
            %length 1
            %T(ni) = ERRORS(ni)*F'*pV;
            T(ni) = F'*pV;
        end
    end
end




