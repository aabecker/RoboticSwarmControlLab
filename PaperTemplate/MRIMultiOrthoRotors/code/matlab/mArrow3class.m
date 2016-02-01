classdef mArrow3class %
    % modifed from http://www.mathworks.com/matlabcentral/fileexchange/25372
    % main change was making it a class.
    methods (Static = true)
        
        function myarrow = mArrow3(p1,p2,varargin)
            %mArrow3 - plot a 3D arrow as patch object (cylinder+cone)
            %
            % syntax:   h = mArrow3(p1,p2)
            %           h = mArrow3(p1,p2,'propertyName',propertyValue,...)
            %
            % with:     p1:         starting point
            %           p2:         end point
            %           properties: 'color':      color according to MATLAB specification
            %                                     (see MATLAB help item 'ColorSpec')
            %                       'stemWidth':  width of the line
            %                       'myarrow.tipWidth':   width of the cone
            %
            %           Additionally, you can specify any patch object properties. (For
            %           example, you can make the arrow semitransparent by using
            %           'facealpha'.)
            %
            % example1: h = mArrow3([0 0 0],[1 1 1])
            %           (Draws an arrow from [0 0 0] to [1 1 1] with default properties.)
            %
            % example2: h = mArrow3([0 0 0],[1 1 1],'color','red','stemWidth',0.02,'facealpha',0.5)
            %           (Draws a red semitransparent arrow with a stem width of 0.02 units.)
            %
            % hint:     use light to achieve 3D impression
            %
            
            propertyNames = {'edgeColor'};
            propertyValues = {'none'};
            
            %% evaluate property specifications
            for argno = 1:2:nargin-2
                switch varargin{argno}
                    case 'color'
                        propertyNames = {propertyNames{:},'facecolor'};
                        propertyValues = {propertyValues{:},varargin{argno+1}};
                    case 'stemWidth'
                        if isreal(varargin{argno+1})
                            myarrow.stemWidth = varargin{argno+1};
                            stemWidth = myarrow.stemWidth;
                        else
                            warning('mArrow3:stemWidth','stemWidth must be a real number');
                        end
                    case 'tipWidth'
                        if isreal(varargin{argno+1})
                            myarrow.tipWidth = varargin{argno+1};
                        else
                            warning('mArrow3:tipWidth','tipWidth must be a real number');
                        end
                    otherwise
                        propertyNames = {propertyNames{:},varargin{argno}};
                        propertyValues = {propertyValues{:},varargin{argno+1}};
                end
            end
            
            %% default parameters
            if ~exist('stemWidth','var')
                ax = axis;
                if numel(ax)==4
                    myarrow.stemWidth = norm(ax([2 4])-ax([1 3]))/300;
                elseif numel(ax)==6
                    myarrow.stemWidth = norm(ax([2 4 6])-ax([1 3 5]))/300;
                end
            end
            if ~exist('myarrow.tipWidth','var')
                myarrow.tipWidth = 3*myarrow.stemWidth;
            end
            myarrow.tipAngle = 22.5/180*pi;
            myarrow.tipLength = myarrow.tipWidth/tan(myarrow.tipAngle/2);
            myarrow.ppsc = 50;  % (points per small circle)
            myarrow.ppbc = 250; % (points per big circle)
            
            %% ensure column vectors
            p1 = p1(:);
            p2 = p2(:);
            
            %% basic lengths and vectors
            x = (p2-p1)/norm(p2-p1); % (unit vector in arrow direction)
            y = cross(x,[0;0;1]);    % (y and z are unit vectors orthogonal to arrow)
            if norm(y)<0.1
                y = cross(x,[0;1;0]);
            end
            y = y/norm(y);
            z = cross(x,y);
            z = z/norm(z);
            
            %% basic angles
            theta = 0:2*pi/myarrow.ppsc:2*pi; % (list of angles from 0 to 2*pi for small circle)
            sintheta = sin(theta);
            costheta = cos(theta);
            upsilon = 0:2*pi/myarrow.ppbc:2*pi; % (list of angles from 0 to 2*pi for big circle)
            sinupsilon = sin(upsilon);
            cosupsilon = cos(upsilon);
            
            %% initialize face matrix
            f = NaN([myarrow.ppsc+myarrow.ppbc+2 myarrow.ppbc+1]);
            
            %% normal arrow
            if norm(p2-p1)>myarrow.tipLength
                % vertices of the first stem circle
                 v = zeros(myarrow.ppsc+1,3);
                for idx = 1:myarrow.ppsc+1
                    v(idx,:) = p1 + myarrow.stemWidth*(sintheta(idx)*y + costheta(idx)*z);
                end
                % vertices of the second stem circle
                p3 = p2-myarrow.tipLength*x;
                for idx = 1:myarrow.ppsc+1
                    v(myarrow.ppsc+1+idx,:) = p3 + myarrow.stemWidth*(sintheta(idx)*y + costheta(idx)*z);
                end
                % vertices of the tip circle
                for idx = 1:myarrow.ppbc+1
                    v(2*myarrow.ppsc+2+idx,:) = p3 + myarrow.tipWidth*(sinupsilon(idx)*y + cosupsilon(idx)*z);
                end
                % vertex of the tiptip
                v(2*myarrow.ppsc+myarrow.ppbc+4,:) = p2;
                
                % face of the stem circle
                f(1,1:myarrow.ppsc+1) = 1:myarrow.ppsc+1;
                % faces of the stem cylinder
                for idx = 1:myarrow.ppsc
                    f(1+idx,1:4) = [idx idx+1 myarrow.ppsc+1+idx+1 myarrow.ppsc+1+idx];
                end
                % face of the tip circle
                f(myarrow.ppsc+2,:) = 2*myarrow.ppsc+3:(2*myarrow.ppsc+3)+myarrow.ppbc;
                % faces of the tip cone
                for idx = 1:myarrow.ppbc
                    f(myarrow.ppsc+2+idx,1:3) = [2*myarrow.ppsc+2+idx 2*myarrow.ppsc+2+idx+1 2*myarrow.ppsc+myarrow.ppbc+4];
                end
                
                %% only cone v
            else
                myarrow.tipWidth = 2*sin(myarrow.tipAngle/2)*norm(p2-p1);
                % vertices of the tip circle
                v = zeros(myarrow.ppbc+1,3);
                for idx = 1:myarrow.ppbc+1
                    v(idx,:) = p1 + myarrow.tipWidth*(sinupsilon(idx)*y + cosupsilon(idx)*z);
                end
                % vertex of the tiptip
                v(myarrow.ppbc+2,:) = p2;
                % face of the tip circle
                f(1,:) = 1:myarrow.ppbc+1;
                % faces of the tip cone
                for idx = 1:myarrow.ppbc
                    f(1+idx,1:3) = [idx idx+1 myarrow.ppbc+2];
                end
            end
            
            %% draw
            fv.faces = f;
            fv.vertices = v;
            myarrow.h = patch(fv);
            for propno = 1:numel(propertyNames)
                try
                    set(myarrow.h,propertyNames{propno},propertyValues{propno});
                catch
                    disp(lasterr)
                end
            end
        end
        
        function updateArrow3(myarrow,p1,p2)
            
            %% ensure column vectors
            p1 = p1(:);
            p2 = p2(:);
            
            %% basic lengths and vectors
            x = (p2-p1)/norm(p2-p1); % (unit vector in arrow direction)
            y = cross(x,[0;0;1]);    % (y and z are unit vectors orthogonal to arrow)
            if norm(y)<0.1
                y = cross(x,[0;1;0]);
            end
            y = y/norm(y);
            z = cross(x,y);
            z = z/norm(z);
            
            %% basic angles
            theta = 0:2*pi/myarrow.ppsc:2*pi; % (list of angles from 0 to 2*pi for small circle)
            sintheta = sin(theta);
            costheta = cos(theta);
            upsilon = 0:2*pi/myarrow.ppbc:2*pi; % (list of angles from 0 to 2*pi for big circle)
            sinupsilon = sin(upsilon);
            cosupsilon = cos(upsilon);
            
            %% initialize face matrix
            f = NaN([myarrow.ppsc+myarrow.ppbc+2 myarrow.ppbc+1]);
            
            %% normal arrow
            if norm(p2-p1)>myarrow.tipLength
                % vertices of the first stem circle
                 v = zeros(myarrow.ppsc+1,3);
                for idx = 1:myarrow.ppsc+1
                    v(idx,:) = p1 + myarrow.stemWidth*(sintheta(idx)*y + costheta(idx)*z);
                end
                % vertices of the second stem circle
                p3 = p2-myarrow.tipLength*x;
                for idx = 1:myarrow.ppsc+1
                    v(myarrow.ppsc+1+idx,:) = p3 + myarrow.stemWidth*(sintheta(idx)*y + costheta(idx)*z);
                end
                % vertices of the tip circle
                for idx = 1:myarrow.ppbc+1
                    v(2*myarrow.ppsc+2+idx,:) = p3 + myarrow.tipWidth*(sinupsilon(idx)*y + cosupsilon(idx)*z);
                end
                % vertex of the tiptip
                v(2*myarrow.ppsc+myarrow.ppbc+4,:) = p2;
                
                % face of the stem circle
                f(1,1:myarrow.ppsc+1) = 1:myarrow.ppsc+1;
                % faces of the stem cylinder
                for idx = 1:myarrow.ppsc
                    f(1+idx,1:4) = [idx idx+1 myarrow.ppsc+1+idx+1 myarrow.ppsc+1+idx];
                end
                % face of the tip circle
                f(myarrow.ppsc+2,:) = 2*myarrow.ppsc+3:(2*myarrow.ppsc+3)+myarrow.ppbc;
                % faces of the tip cone
                for idx = 1:myarrow.ppbc
                    f(myarrow.ppsc+2+idx,1:3) = [2*myarrow.ppsc+2+idx 2*myarrow.ppsc+2+idx+1 2*myarrow.ppsc+myarrow.ppbc+4];
                end
                
                %% only cone v
            else
                myarrow.tipWidth = 2*sin(myarrow.tipAngle/2)*norm(p2-p1);
                % vertices of the tip circle
                v = zeros(myarrow.ppbc+1,3);
                for idx = 1:myarrow.ppbc+1
                    v(idx,:) = p1 + myarrow.tipWidth*(sinupsilon(idx)*y + cosupsilon(idx)*z);
                end
                % vertex of the tiptip
                v(myarrow.ppbc+2,:) = p2;
                % face of the tip circle
                f(1,:) = 1:myarrow.ppbc+1;
                % faces of the tip cone
                for idx = 1:myarrow.ppbc
                    f(1+idx,1:3) = [idx idx+1 myarrow.ppbc+2];
                end
            end
            set(myarrow.h, 'faces', f, 'vertices', v)
        end
    end
end