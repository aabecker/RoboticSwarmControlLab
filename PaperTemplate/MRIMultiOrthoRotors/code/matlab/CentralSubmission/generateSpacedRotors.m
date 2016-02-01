function Rs =  generateSpacedRotors(n)
    % generates n well-spaced axes for n rotors.  Uses function generateNonParallelAxes
    % returns Rs, the rotation matrices about each rotor's axis
    Rs = cell(n,1);
    
    v = generateNonParallelAxes(n);  %n axis, defined as a 3xn matrix of [x,y,z] values
    nomAxisOfRotation = [0;0;1];  %nominal axis is the z-axis
    for ni = 1:n
        
        theta = acos(dot( nomAxisOfRotation,v(ni,:))/  (norm(nomAxisOfRotation)*norm(v(ni,:)) ) );
        if theta < eps   %two axes are parallel
            Rs{ni} = eye(3);
        else
            if pi-theta < eps
                x = [1;0;0];  % the two axes are 180 degrees apart
            else
                x = cross(nomAxisOfRotation,v(ni,:))/norm(cross(nomAxisOfRotation,v(ni,:)));
            end
            Rs{ni} = rotateAboutAxisTheta(x(1),x(2),x(3), theta);
        end
    end
    
    % % Display the rotors
    %         figure(400)
    %         clf
    %         for nj = 1:n
    %             a = Rs{nj}*nomAxisOfRotation;
    %             plot3(a(1),a(2),a(3),'b.','MarkerSize',18)
    %             hold on
    %             plot3(v(nj,1),v(nj,2),v(nj,3),'ro','MarkerSize',14)
    %
    %
    %             plot3([0,a(1)],[0,a(2)],[0,a(3)],'k')
    %         end
    %         axis equal
    %         axis tight
    
    
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


