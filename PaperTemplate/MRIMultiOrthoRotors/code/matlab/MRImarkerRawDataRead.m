function MRImarkerRawDataRead()
    % Data Format:
    % xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
    % |cha1|cha2|...|cha12|cha1|cha2|...|cha12|cha1...
    % |prjX               |prjY               |prjX
    % |acquisition 1                          |acquisition 2 ...
    
    % TODO: find a representative x and y image and plot them.
    %       plot the x-y positions of the rotor for all 30 scans
    
    fnames = {
        'meas_rotorSpinningMarker0Hz_12CHA_1024Samples_30Prj'  %not moving
        'meas_rotorSpinningMarker0.25Hz'
        'meas_rotorSpinningMarker0.5Hz'
        'meas_rotorSpinningMarker1Hz'
        'meas_rotorSpinningMarker1.5Hz'
        'meas_rotorSpinningMarker2Hz' %motor stopped rotating ( I think )
        
        'meas3RotorsOpenLoopVer1_12Cha_512Samples_100Prj'; %#7. 
        'meas3RotorsOpenLoopVer2'; %#8, 
        'meas3RotorsOpenLoopVer3';};  %#9, 
    
    fileNum = 6;
    if fileNum < 7
        %Parameters to be used for 1 rotor file
        NbAcquisition = 30; %30 for 1 rotor
        NbChannels = 12;
        NbSamples = 1024; % 1024 for 1 rotor
        npeaks = 1;
    else
        % parameters to be used with :
        % meas3RotorsOpenLoopVer1, meas3RotorsOpenLoopVer2, meas3RotorsOpenLoopVer3
        NbAcquisition = 100;
        NbChannels = 12;
        NbSamples = 512;
        npeaks = 3;
    end
    
    fid = fopen(fnames{fileNum});
    
    prjX = zeros(NbAcquisition,NbChannels, NbSamples);
    prjY = zeros(NbAcquisition,NbChannels, NbSamples);
    sc = 300/NbSamples;  %scaling from sampled number to mm
    
    peaksX = zeros(NbAcquisition,npeaks);
    peaksY = zeros(NbAcquisition,npeaks);
    
    for i = 1:NbAcquisition
        %         f1= figure(1);
        %         clf
        %set(f1,'position',[3 9 857 988]);
        %         set(f1,'name',sprintf('x-projection %d/%d, %s',i,NbAcquisition,fnames{fileNum}));
        for j = 1:NbChannels
            prjX(i,j,:) = fread(fid, NbSamples, 'float');
            %             subplot(4,3,j)
            %             plot(squeeze(prjX(i,j,:)),'linewidth',2)
            %
            %             hold on
            %             axis tight
            %
            %              [xout,yout,peakspos] = peakfind(1:NbSamples,squeeze(prjX(i,j,:)),2,5,5);
            %              [~,ind] = sort(-yout(peakspos));
            %              plot(xout,yout,'r')
            %              maxInds = peakspos(ind(1:npeaks));
            %              plot(xout(maxInds),yout(maxInds),'go','Markersize',10,'linewidth',2)
            %
            %             t = sprintf('Channel %d/%d',j,NbChannels);
            %             %display(t);
            %             title(t);
        end
        f3= figure(3);
        clf
        set(f3,'name',sprintf('x-projection %d/%d, %s',i,NbAcquisition,fnames{fileNum}));
        pX = sum(squeeze(prjX(i,:,:)));
        positX = sc*(1:NbSamples)' - 150;
        plot(positX,pX,'linewidth',2)
        hold on
        axis tight
        [Xxout,Xyout,peakspos] = peakfind(positX,pX,2,8,5);
        [~,ind] = sort(-Xyout(peakspos));
        plot(Xxout,Xyout,'r')
        maxIndsX = sort(peakspos(ind(1:npeaks)));
        plot(Xxout(maxIndsX),Xyout(maxIndsX),'go','Markersize',10,'linewidth',2)
        xlabel('mm')
        ylabel('inverse FFT')
        peaksX(i,:) = Xxout(maxIndsX);
        
        %         f2 = figure(2);
        %         clf
        %         set(f2,'name',sprintf('y-projection %d/%d %s',i,NbAcquisition, fnames{fileNum}));
        for j = 1:NbChannels
            prjY(i,j,:) = fread(fid, NbSamples, 'float');
            %             subplot(4,3,j)
            %             plot(squeeze(prjY(i,j,:)),'linewidth',2)
            %             hold on
            %             %t = sprintf('Channel %d/%d',j,NbChannels);
            %              [xout,yout,peakspos] = peakfind(1:NbSamples,squeeze(prjY(i,j,:)),2,5,5);
            %              [~,ind] = sort(-yout(peakspos));
            %              maxInds = peakspos(ind(1:npeaks));
            %              plot(xout(maxInds),yout(maxInds),'go','Markersize',10,'linewidth',2)
            %             title(t);
            %             axis tight
        end
        f4= figure(4);
        clf
        set(f4,'name',sprintf('Y-projection %d/%d, %s',i,NbAcquisition,fnames{fileNum}));
        pY = sum(squeeze(prjY(i,:,:)));
        positY = sc*(1:NbSamples)' - 150;
        plot(positY,pY,'linewidth',2)
        hold on
        axis tight
        [Yxout,Yyout,peakspos] = peakfind(positY,pY,2,8,5);
        [~,ind] = sort(-Yyout(peakspos));
        plot(Yxout,Yyout,'r')
        maxIndsY = sort(peakspos(ind(1:npeaks)));
        plot(Yxout(maxIndsY),Yyout(maxIndsY),'go','Markersize',10,'linewidth',2)
        xlabel('mm')
        ylabel('inverse FFT')
        peaksY(i,:) = Yxout(maxIndsY);
    end
    
    
    
    %Save data for plotting
    repXx = Xxout;
    repXy = Xyout;
    repYx = Yxout;
    repYy = Yyout;
    
    
    figure(7)
    plot(peaksX,peaksY,'.')
    axis equal
    
    save( ['MRIdata/', fnames{fileNum}, 'Peaks.mat'], 'peaksX', 'peaksY', 'repXx', 'repXy', 'repYx', 'repYy','maxIndsY','maxIndsX','positX','pX','positY','pY' );
    fclose(fid);