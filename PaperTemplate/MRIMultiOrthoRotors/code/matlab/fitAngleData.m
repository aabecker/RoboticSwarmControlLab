function fitAngleData        

        savefilename = 'MultiAxisCalc/MultiAxisMinimumAngle.mat';
        load(savefilename,'MIN_ANGLES')
        nC = numel(MIN_ANGLES);
        means = zeros(1,nC);
        for i = 1:nC
            means(i) = mean( MIN_ANGLES{i} );
        end
        
        function F = invFunc(a,x)
            F = a(1)*0+a(2)./x;
        end
        
        function F = invSqFunc(a,x)
            F = a(1)+a(2)./x+ a(3)./(x.^2)
        end
        
        function F = invSqFuncLin(a,x)
            F = a(1)+a(2)./x+ a(3)./(x.^2)+a(4).*x;
        end
        
        figure(2)
        clf
        set(gcf,'name', [ num2str(numel(MIN_ANGLES{1})),' samples'])
        
        x = 2:nC;
        y = means(x);
        [a1,resnorm1] = lsqcurvefit(@invFunc,[1,1],x,y)
        [a2,resnorm2] = lsqcurvefit(@invSqFunc,[a1,0],x,y);
        [a3,resnorm3] = lsqcurvefit(@invSqFuncLin,[a2,0],x,y);
        hold on
        
        x = 5:nC;
        y = means(x);
        
        plot(x,y,'o') %plot data
        plot(x,invFunc(a1,x),'r') %fit
        plot(x,invSqFunc(a2,x),'m') %fit
        plot(x,invSqFuncLin(a3,x),'g') %fit
        display([resnorm1,resnorm2,resnorm3])
        xlabel('number of rotors $n$')
        ylabel('minimum seperation angle [deg]')
%         figure(3)
%         clf
%         a=lsqcurvefit(@(a,x) a(1)./x,1,x,y);
% hold on
% plot(x,y,'o') %plot data
% plot(x,a./x) %fit
%         
end