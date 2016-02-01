function MRImarkerPlot()
    % Data Format:
    % xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
    % |cha1|cha2|...|cha12|cha1|cha2|...|cha12|cha1...
    % |prjX               |prjY               |prjX
    % |acquisition 1                          |acquisition 2 ...
    
    % TODO: find a representative x and y image and plot them.
    %       plot the x-y positions of the rotor for all 30 scans
    set(0,'DefaultAxesFontSize',14)
set(0,'defaultaxesfontsize',14);

    fnames = {
        'meas_rotorSpinningMarker0Hz_12CHA_1024Samples_30Prj'
        'meas_rotorSpinningMarker0.25Hz'
        'meas_rotorSpinningMarker0.5Hz'
        'meas_rotorSpinningMarker1Hz'
        'meas_rotorSpinningMarker1.5Hz'
        'meas_rotorSpinningMarker2Hz' %motor stopped rotating ( I think )
        
        'meas3RotorsOpenLoopVer1_12Cha_512Samples_100Prj'; %#7.
        'meas3RotorsOpenLoopVer2'; %#8,
        'meas3RotorsOpenLoopVer3';};  %#9,
    
    
    %     if fileNum < 7
    %         %Parameters to be used for 1 rotor file
    %         NbAcquisition = 30; %30 for 1 rotor
    %         NbChannels = 12;
    %         NbSamples = 1024; % 1024 for 1 rotor
    %         npeaks = 1;
    %     else
    %         % parameters to be used with :
    %         % meas3RotorsOpenLoopVer1, meas3RotorsOpenLoopVer2, meas3RotorsOpenLoopVer3
    %         NbAcquisition = 100;
    %         NbChannels = 12;
    %         NbSamples = 512;
    %         npeaks = 3;
    %     end
    
    xrange = [-67,66];
    yrange = [-70,16];
    color = ['y','b','g','c','r','k','m','r','b','m','r','b','m','r','b','m','r','b','m'];
    figure(8)
    clf
    n(7) = 1;
    n(9) = 2;
    for i = 7:9 %1:6
        load(['MRIdata/',fnames{i}, 'Peaks.mat']);%, 'peaksX', 'peaksY', 'repXx', 'repXy', 'repYx', 'repYy' );
        subplot(3,4,[2,3,4,6,7,8])
        %plot(peaksX,peaksY,[color(i),'.'])
        plot(peaksX,peaksY,['r','.'])
        axis equal
        hold on
        axis([xrange(1), xrange(2),yrange(1), yrange(2)])
        set(gca,'Xticklabel',[],'Yticklabel',[])
        
        if i==7 ||i==9
            for j = 1:numel(maxIndsX)
                plot( [repXx(maxIndsX(j)),repXx(maxIndsX(j))],[-1000,repYx(maxIndsY(j))],color(i))
                hold on
                plot( [-1000,repXx(maxIndsX(j))],[repYx(maxIndsY(j)),repYx(maxIndsY(j))],color(i))
                plot( repXx(maxIndsX(j)),repYx(maxIndsY(j)),'*','color',color(i))
            end
            
            subplot(3,4,[1,5])
            plot(pY,positY,'linewidth',2,'color',color(i))
            hold on
            %plot(repYy,repYx,'r')plot(repYy(maxIndsY),repYx(maxIndsY),'go','Markersize',10,'linewidth',2)
            ylabel('y [mm]')
            axis([0,0.01,yrange(1), yrange(2)])
            
            subplot(3,4,[10,11,12])
            plot(positX,pX,'linewidth',2,'color',color(i))
            text(repXx(maxIndsX(1)),repXy(maxIndsX(1)),['T_',num2str(n(i)),' '],'color',color(i),'HorizontalAlignment','right','VerticalAlignment','bottom')
            hold on
            %plot(repXx,repXy,'r')plot(repXx(maxIndsX),repXy(maxIndsX),'go','Markersize',10,'linewidth',2)
            xlabel('x [mm]')
            axis([xrange(1), xrange(2),0,0.01])
        end
        
    end

    set(gcf,'papersize',[7,5])
set(gcf,'paperposition',[0,0,7,5])
print -dpdf 'MarkerRead3rotors.pdf'
%print -dpdf '../../pictures/pdf/MarkerRead.pdf'    
    