function MultiAxisCalcTorquePLOT
    % plots data from MultiAxisCalcTorque
    
    dffontsz= 14;
    set(0,'DefaultLineLineWidth',2)
    set(0,'defaultaxesfontsize',dffontsz);
    set(0,'defaulttextfontsize',dffontsz);
    format compact
    set(0,'DefaultTextInterpreter', 'latex')
    
    
    %1.  plot the average totaltorque & variance as a function of n
    %2. plot the average individual torque & variance as a function of n
    
    %savefilename = 'MultiAxisCalcTorqueData.mat';
    savefilename = 'MultiAxisCalcTorqueData1M.mat';
    savefilename = 'MultiAxisCalcTorqueData1Mfixed.mat';
    % {[0.2472, 0.24, 0.6], [0.6, 0.24, 0.442893],
    %  [0.6, 0.547014, 0.24], [0.24, 0.6, 0.33692],
    %  [0.24, 0.353173, 0.6], [0.6, 0.24, 0.563266],
    %  [0.6, 0.426641, 0.24], [0.263452, 0.6, 0.24],
    %  [0.24, 0.473545, 0.6], [0.516361, 0.24, 0.6]}
    mycolor = [0,0,1];
    
    figure(1)
    clf
    figure(2)
    clf
    plotTorques(savefilename,[0.2472, 0.24, 0.6]) %http://stackoverflow.com/questions/5391825/what-are-the-standard-colors-for-plots-in-mathematica
    plotTorques('MultiAxisCalcTorqueDataRandAxis.mat',[0.6, 0.24, 0.442893])
    plotTorques('MultiAxisCalcTorqueDataSameAxis.mat',[0.6, 0.547014, 0.24])
    
    figure(1)
    format_ticks(gca,{'0','5','10','15','20','25'},{'0','2','4','6','8'},0:5:25,0:2:8);
    xlabel({'';'';'$n$, Number of rotors'})
    ylabel({'Average Sum Stopped Torque';''})
    text(1,8.5,  'Rotor Placement:' )
    text(2,7.75,  'Optimized', 'color',[0.2472, 0.24, 0.6])
    text(2,7.,  '$\phi,\lambda$=Rand[0,1]', 'color',[0.6, 0.24, 0.442893])
    text(2,6.25,  'All $z$-axis', 'color',[0.6, 0.547014, 0.24])
    
    set(gcf,'PaperUnits','inches')
    set(gcf,'papersize',[4,4])
    set(gcf,'paperposition',[-0.7,0,5,4])
    print -dpdf '../../pictures/pdf/AveStoppedTorqueSum'
    
    figure(2)
    format_ticks(gca,{'0','5','10','15','20','25'},{'0','0.5','1.0','1.5'},0:5:25,0:.5:1.5);
    xlabel({'';'';'$n$, Number of rotors'})
    ylabel({'Average Individual Stopped Torque';'';''})
    text(11,.8,  'Rotor Placement:' )
    text(12,.7,  'Optimized', 'color',[0.2472, 0.24, 0.6])
    text(12,.6,  '$\phi,\lambda$=Rand[0,1]', 'color',[0.6, 0.24, 0.442893])
    text(12,.5,  'All $z$-axis', 'color',[0.6, 0.547014, 0.24])
    
    set(gcf,'PaperUnits','inches')
    set(gcf,'papersize',[4,4])
    set(gcf,'paperposition',[-0.7,0,5,4])
    print -dpdf '../../pictures/pdf/AveStoppedTorqueInd'
    
    
    function plotTorques(savefilename,mycolor)
        savefilename = ['MultiAxisCalc/',savefilename];
        if exist(savefilename, 'file') == 2
            load(savefilename,'NUM_TESTS_Total', 'TORQUES_MEAN','TORQUES_MIN','TORQUES_MAX','TORQUES_VAR','TORQUES_ROWSUM_MEAN','TORQUES_ROWSUM_VAR','TORQUES_ROWSUM_AVE' )
            
            nMax = numel(TORQUES_ROWSUM_AVE);
            figure(1)
            plot(TORQUES_ROWSUM_MEAN,'-o','color',mycolor,'linewidth',2)
            hold on
            stdVals = sqrt(TORQUES_ROWSUM_VAR);
            p1 =patch([1:1:nMax, nMax:-1:1],[TORQUES_ROWSUM_MEAN+stdVals,fliplr(TORQUES_ROWSUM_MEAN-stdVals)],2/3*mycolor+1/3);
            set(p1,'line','none','FaceAlpha',1)
            uistack(p1,'bottom')
            plot(TORQUES_ROWSUM_MEAN+stdVals,'-','color',1/2*mycolor+1/2,'linewidth',1);
            % plot(TORQUES_ROWSUM_MEAN-sqrt(TORQUES_ROWSUM_VAR),'-or','linewidth',2)
            % plot(TORQUES_ROWSUM_MEAN+sqrt(TORQUES_ROWSUM_VAR),'-or','linewidth',2)
            % plot(TORQUES_MAX,'-og','linewidth',2)  % can't plot this
            %  plot(TORQUES_MIN,'-og','linewidth',2)
            
            figure(2)
            plot(TORQUES_ROWSUM_AVE,'-o','color',mycolor,'linewidth',2);
            hold on
            % for n = 1:nMax  %plot the mean for each rotor
            %   plot(n*ones(n,1), TORQUES_MEAN{n},'r.')
            % end
            % Plot the average standard deviation
            meanStd = zeros(1,nMax);
            for n = 1:nMax  %plot the mean for each rotor
                meanStd(n) = mean(TORQUES_VAR{n}); %#ok<USENS>
            end
            xs = [1:1:nMax, nMax:-1:1];
            ys = max(0,[TORQUES_ROWSUM_AVE+meanStd,fliplr(TORQUES_ROWSUM_AVE-meanStd)]);
            p2 = patch(xs,ys,1/2*mycolor+1/2);
            plot(TORQUES_ROWSUM_AVE+meanStd,'-','color',1/2*mycolor+1/2,'linewidth',1);
            set(p2,'line','none','FaceAlpha',1)
            %     p2 = patch(xs,ys,2/3*mycolor+1/3);
            %     set(p2,'line','none','FaceAlpha',0.5)
            uistack(p2,'bottom')
            %axis([0,25,0,1.5])
            
            %  plot(TORQUES_ROWSUM_AVE+meanStd(n),'-or','linewidth',2)
            %   plot(TORQUES_ROWSUM_AVE-meanStd(n),'-or','linewidth',2)
            %     plot(TORQUES_ROWSUM_AVE-TORQUES_ROWSUM_VAR,'-o','linewidth',2)
            %     plot(TORQUES_ROWSUM_AVE+TORQUES_ROWSUM_VAR,'-o','linewidth',2)
            
            
        end
        
        
        
    end
    
    
    % %%  Do a lot of simulations!
    % tic
    % for n = 1:25
    %    display(['n = ',num2str(n),'toc =',num2str(toc)])
    %     MultiAxisCalcTorque(n,1000000,'MultiAxisCalcTorqueData1M.mat')
    % end
    % tic
    % for n = 1:25
    %    display(['n = ',num2str(n),', toc =',num2str(toc)])
    %     MultiAxisCalcTorque(n,1000000,'MultiAxisCalcTorqueData1Mfixed.mat')
    % end
    
    % %% simulate with non-optimal spacing
    % tic
    % for n = 1:25
    %    display(['n = ',num2str(n),'toc =',num2str(toc)])
    %     MultiAxisCalcTorque(n,1000000,'MultiAxisCalcTorqueDataRandAxis.mat')
    % end
    
    % %% simulate with no spacing
    % tic
    % for n = 1:25
    %    display(['n = ',num2str(n),'toc =',num2str(toc)])
    %     MultiAxisCalcTorque(n,1000000,'MultiAxisCalcTorqueDataSameAxis.mat')
    % end
    
    
end