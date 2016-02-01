function MultiAxisMinimumAngle()
    % what is the minimum angle between any two rotors, given n rotors?
    % this code runs generateSpacedRotors.m to find a locally optimal
    % spacing for n rotors, then finds the minimum angle. 
    %  We run this multiple times for different n values, then plot the average minimum angle +/- the 
    % minimums detected.  We then can run fitAngleData to determine a
    % relationship between n and minimum angle.  Note that we are finding
    % local minmums, or we are optimizing on the energy, not the minimum
    % angle...
    nMax = 100;
    format compact
    savefilename = 'MultiAxisCalc/MultiAxisMinimumAngle.mat';
    
    
    if exist(savefilename, 'file') == 2
        load(savefilename,'MIN_ANGLES')
    else
        MIN_ANGLES = cell(1,nMax);
    end
    
    if numel(MIN_ANGLES) < nMax
        for i = numel(MIN_ANGLES):nMax
            MIN_ANGLES{i} = [];
        end
    end
    
    %simulate rotors
    for reps = 1:5
        for n = 1:nMax
            Rs =  generateSpacedRotors(n);
            minA = distanceBetweenRotationMatrices(Rs);
            
            MIN_ANGLES{n}(numel(MIN_ANGLES{n})+1) = minA;
        end
        save(savefilename, 'MIN_ANGLES' )
    end
    
    
    
    nC = numel(MIN_ANGLES);
    means = zeros(1,nC);
    stds = zeros(1,nC);
    for i = 1:nC
        means(i) = mean( MIN_ANGLES{i} );
        stds(i) = std( MIN_ANGLES{i} );
    end
    
    figure(100)
    clf
    set(gcf,'name', [ num2str(numel(MIN_ANGLES{1})),' samples'])
    mycolor = [0.2472, 0.24, 0.6]; %http://stackoverflow.com/questions/5391825/what-are-the-standard-colors-for-plots-in-mathematica
    
    plot(means,'-o','color',mycolor,'linewidth',2);
    hold on
    
    
    xs = [1:1:nC, nC:-1:1];
    ys = max(0,[means+3*stds,fliplr(means-3*stds)]);
    p2 = patch(xs,ys,1/2*mycolor+1/2);
    plot(means+stds,'-','color',1/2*mycolor+1/2,'linewidth',1);
    set(p2,'line','none','FaceAlpha',1)
    %     p2 = patch(xs,ys,2/3*mycolor+1/3);
    %     set(p2,'line','none','FaceAlpha',0.5)
    uistack(p2,'bottom')
    
     format_ticks(gca,{'0','5','10','15','20','25'},{'0','50','100','150','200'},0:5:25,0:50:200);
        xlabel({'';'';'$n$, Number of rotors'})
        ylabel({'Minimum Average Angle +/-3 standard deviations [deg]';''})
    
    
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
        %         display(minA)
        %         display(angs)
    end
    
    function phi = minAngle2Rot(Ra,Rb)
        %minimum angle between two rotation matrices, a type of metric
        phi = acos(  ( trace( Ra'*Rb)-1 )/2 );
    end
    
end



%%
Rs =  generateSpacedRotors(500);
minA = distanceBetweenRotationMatrices(Rs);


