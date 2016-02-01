function v = generateNonParallelAxes(n)
% Generates n axes-of-rotation that are _well-spaced_ , that is all axes are
% as far from being parallel as possible.  The resulting axes minimize
% clustering.
%
% Returns v, the $n\times3$ vectors in 3-space of these rotation axes, all in the 
% upper hemisphere
%
% This is a generalization of the Thomson problem (J.J. Thomson 1904). The
% Thomson problem determines the minimum energy configuration for n 
% electrons confined to the surface of a sphere: 
% http://en.wikipedia.org/wiki/Thomson_problem
%
% To generate well-spaced axes, additional constraints are imposed by _coupling_
% each electron with an additional electron at its antipodal point, i.e.
% $x$ and $-x$ in 3-space.
% The solution generates _maximally_ separated directions in 3-space, where maximally
% means that the directions are as far from being parallel as possible.
%
%  Aaron Becker, |aabecker@gmail.com| 11/26/2013
%
% based on code from |penghao888@gmail.com| on behalf of Hao Peng [pengh@purdue.edu]
% and their unpublished “Optimization on the surface of a hyper sphere”,
% https://www.cs.purdue.edu/homes/pengh/reports/590OP.pdf
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if(nargin <1)
    n=100;  %number of 'minimally parallel' or 'maximumly independent' axis to generate.
end
global functionTime;
% Some regular shapes we generate:
% square for n = 2
% Octahedron for n = 3
%  hexahedron for n = 4
% Pentagonal antiprism   n = 5   
% Icosahedron for n = 6
% Gyroelongated hexagonal bipyramid for n = 7   (not regular)
% n = 12, gives 24 vertices, 8 squares along equator, 2 squares top and bottom, 24
%  triangles = 34 Faces,  10{4}+24{3}

format compact
set(0,'defaultaxesfontsize',16);
set(0,'defaulttextfontsize',16);

functionTime = 0;
% Axis are in spherical Coordinates
x0 = rand(1,2*n);  %first n are lat (elevation), next n are longitude (azimuth)
options = optimset('MaxIter', 10000, 'MaxFunEvals', 10000, 'GradObj', 'on','TolFun',1e-8,'Algorithm','trust-region-reflective');
%OPTIONS parameter Algorithm:
% must be 'active-set', 'trust-region-reflective' (default), 'interior-point', 'interior-point-convex', 'levenberg-marquardt',
% 'trust-region-dogleg', 'lm-line-search', 'sqp', or 'simplex'.  %noe seem
% to generate exactly the right values for n=1,2,3

time = cputime;

% %%%optional plotting  (animates the process)
% set(0,'DefaultAxesFontSize',18)
% MOVIE_NAME = 'CreateIndependentAxisFast';
% G.fig = figure(2);
% set(G.fig,'Units','normalized','outerposition',[0 0 1 1],'NumberTitle','off');%'MenuBar','none',
% writerObj = VideoWriter(MOVIE_NAME,'MPEG-4');%http://www.mathworks.com/help/matlab/ref/videowriterclass.html
% set(writerObj,'Quality',100)
% open(writerObj);
% [x,fval,exitflag,output,grad] = fminunc(@(x) objectiveNonParallelAxes(x, n, writerObj), x0, options);


% starts at the point x0 and attempts to find a local minimum x of the
% function described in objectiveNonParallelAxes.
[x,fval,~,output,~] = fminunc(@(x) objectiveNonParallelAxes(x, n), x0, options);
time = cputime - time;
fprintf('iterations: %d,  funcCount %d\n', output.iterations, output.funcCount);
fprintf('energy: %e\n', fval);
fprintf('time used: %f\n', time);
fprintf('average function call time: %f\n', functionTime/output.funcCount);

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

%%% Plot the final configuration
[X, Y, Z] = sph2cart(t, r, 1);  %[x,y,z] = sph2cart(azimuth,elevation,r)
[X2, Y2, Z2] = sph2cart(t, r+pi, 1);
figure(1)
clf
subplot(1,2,1)
plot3(X, Y, Z,'b.','MarkerSize',18)
hold on
plot3(X2, Y2, Z2,'r.','MarkerSize',18)
for ci=1:n
    plot3([X(ci),X2(ci)], [Y(ci),Y2(ci)], [Z(ci),Z2(ci)],'k-','linewidth',2)
end
axis square;
title(['\it n\rm = ',num2str(n),' non-parallel axes'])
grid on
set(gca,'XTick',[-1,0,1],'YTick',[-1,0,1],'ZTick',[-1,0,1]);

subplot(1,2,2)

plot3(X, Y, Z,'b.','MarkerSize',18)
hold on
plot3(X2, Y2, Z2,'r.','MarkerSize',18)
sc = 0.92;
if n < 3
    patch(sc*[X,X2], sc*[Y,Y2], sc*[Z,Z2],[0.8,0.5,0.5],'FaceLighting','flat');
else  % less than 3, may be coplanar
    k = convhull([X;X2], [Y;Y2], [Z;Z2]);
    trimesh(k,sc*[X;X2], sc*[Y;Y2], sc*[Z;Z2],'FaceColor',[0.8,0.5,0.5],'FaceLighting','flat');
end
title('Hull')
camlight right
colormap([1,0,0;0,0,1])
set(gca,'XTick',[-1,0,1],'YTick',[-1,0,1],'ZTick',[-1,0,1]);
axis square;
%colormap(gray);
%axis vis3d;
%title(['\it n\rm = ',num2str(n),' non-parallel axes'])
grid on

v = [X', Y', Z'];

    function [f,g] = objectiveNonParallelAxes(x, n)
        % objectiveNonParallelAxes is a function that accepts a vector x
        %and returns a scalar f, the objective function evaluated at x; and g, the
        %gradient of the objective function
        %
        % Used to Generate n axes-of-rotation that are 'well-spaced', that is all axes are
        % as far from being parallel as possible.
        % INPUTS: n is number of axes to create
        %         x is the coordinates of points in spherical coordinates 
        % This is a generalization of The Thomson problem 
        %
        %  Aaron Becker, aabecker@gmail.com 11/26/2013
        %
        % based on code from penghao888@gmail.com on behalf of Hao Peng [pengh@purdue.edu]
        % and their unpublished “Optimization on the surface of a hyper sphere”,
        % https://www.cs.purdue.edu/homes/pengh/reports/590OP.pdf
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %global functionTime;
        functionTime = functionTime - cputime;
        r = x(1:n); %first n are lat (elevation),  Adding pi to r will get the antipode
        t = x(n+1:2*n); % next n are longitude (azimuth)
        
        
        % %%%% %optional plotting  (animates the process)
        % persistent FrameCount;
        % if isempty(FrameCount)
        %     FrameCount = 0;
        % end
        % FrameCount=FrameCount+1
        % if mod(FrameCount-1,100) == 1
        %     ind = r>pi;
        %     while sum(ind)
        %         r(ind) = r(ind)-pi;
        %         ind = r>pi;
        %     end
        %     ind = r<0;
        %     while sum(ind)
        %         r(ind) = r(ind)+pi;
        %         ind = r<0;
        %     end
        %     t = x(n+1:2*n);
        %     [X, Y, Z] = sph2cart(t, r, 1);  %[x,y,z] = sph2cart(azimuth,elevation,r)
        %     [X2, Y2, Z2] = sph2cart(t, r+pi, 1);
        %     figure(2)
        %     clf
        %     %k = convhull(X, Y, Z);
        %     %trimesh(k, X, Y, Z);
        %     plot3(X, Y, Z,'b.','MarkerSize',18)
        %     hold on
        %     plot3(X2, Y2, Z2,'r.','MarkerSize',18)
        %     for i=1:n
        %         plot3([X(i),X2(i)], [Y(i),Y2(i)], [Z(i),Z2(i)],'k-','linewidth',2)
        %     end
        %     grid on
        %     axis square;
        %     set(gca,'XTick',[-1,0,1],'YTick',[-1,0,1],'ZTick',[-1,0,1]);
        %     set(gca,'gridlinestyle','-')
        %     %drawnow
        %     %pause(0.1)
        %     updateDrawing
        % end
        % %%%%%%%%%%%%%%%%%%%%%%%%%%%% END OPTIONAL PLOTTING %%%%%%%%%%%%%%%%%%%%
        
        % VECTORIZED CODE:     
        %Comparison tests for n = 50 show a 10x speedup:
        % non-vectorized, in (s) :  26.4, 19.09, 23
        % vectorized, in (s)     :  2.78, 2.97, 2.48
        % Precompute
        mycosr = cos(r);
        mysinr = sin(r);
        mycost = cos(repmat(t,n,1)-repmat(t',1,n));
        mysint = sin(repmat(t',1,n)-repmat(t,n,1));
        
        denom=repmat(mycosr,n,1).*repmat(mycosr',1,n).*mycost+repmat(mysinr,n,1).*repmat(mysinr',1,n);
        tmp = 1/2./(1-denom);
        tmp2 = 1/2./(1+denom);
        tmp(1:n+1:n*n) = 0;
        tmp2(1:n+1:n*n) = 0;
        
        % compute f, the objective function
        f = sum(sum(triu(tmp)+tril(tmp2)));
        
        % compute g, the gradient
        denom2 = (tmp2.^2-tmp.^2);
        %Gradient with respect to Elevation (latitude) coordinate
        gradE = (repmat(mysinr,n,1).*repmat(mycosr',1,n).*mycost-repmat(mycosr,n,1).*repmat(mysinr',1,n))*2.*denom2;
        gradE(1:n+1:n*n) = 0;
        gE = sum(gradE);
        %Gradient with respect to Azimuth (longitude) coordinate
        gradA=(repmat(mycosr,n,1).*repmat(mycosr',1,n).*mysint)*-2.*denom2;
        gradA(1:n+1:n*n) = 0;
        gA = sum(gradA);
        g = [gE',gA'];
       

%       %NON-VECTORIZED  (suitable for C implementation)
%         %precompute
%         mycosr = cos(r);
%         mysinr = sin(r);
%         mycost = zeros(n, n);  
%         mysint = zeros(n, n);
%         for i = 1:n
%             for j = 1:i-1
%                 mycost(i,j) = cos(t(i) - t(j));
%                 mycost(j,i) = mycost(i,j);
%                 mysint(i,j) = sin(t(i) - t(j));
%                 mysint(j,i) = -mysint(i,j);
%             end
%         end
%         
%         tmp = zeros(n,n); %pre-compute
%         tmp2 = zeros(n,n); %pre-compute
%         for j = 1:n
%             for i = 1:j-1
%                 denom = mycosr(i)*mycosr(j)*mycost(i,j)+mysinr(i)*mysinr(j);
%                 tmp(i,j) = 1/2/(1-denom);
%                 tmp(j,i) = tmp(i,j);
%                 tmp2(i,j) = 1/2/(1+denom);
%                 tmp2(j,i) = tmp2(i,j);
%             end
%         end
% 
%         % compute f, the objective function
%         f=0;
%         for j = 1:n
%             for i = 1:j-1
%                 f = f+tmp(i,j)+tmp2(i,j);
%             end
%         end
% 
%         % compute g, the gradient
%         g = zeros(2*n,1);
%         for i = 1:n
%             for j = 1:n
%                 if j ~= i   %Gradient with respect to Elevation (latitude) coordinate
%                     geoTerm = (mysinr(i)*mycosr(j)*mycost(i,j)-mycosr(i)*mysinr(j))*2;
%                     g(i) = g(i)+geoTerm*(-tmp(i,j)^2 + tmp2(i,j)^2);
%                 end
%             end
%         end
%         for i = 1:n
%             for j = 1:n
%                 if j ~= i  %Gradient with respect to Azimuth (longitude) coordinate
%                     geoTerm = (mycosr(i)*mycosr(j)*mysint(i,j))*2;
%                     g(n+i) = g(n+i)+geoTerm*(-tmp(i,j)^2+tmp2(i,j)^2);
%                 end
%             end
%         end

        
        functionTime = functionTime + cputime;
        
        
        
        %     function updateDrawing
        %     %%% Make a video of the convergence
        %         drawnow
        %         figure(2)
        %         title(['Step ',num2str(ceil(FrameCount/100))])
        %         tfig = myaa(4);
        %         F = getframe;
        %         writeVideo(writerObj,F.cdata);
        %         if(FrameCount == 100)
        %             for ic =1:10
        %                 writeVideo(writerObj,F.cdata);
        %             end
        %         end
        %         close(tfig)
        %
        %
        %     end
        
        
    end
end

