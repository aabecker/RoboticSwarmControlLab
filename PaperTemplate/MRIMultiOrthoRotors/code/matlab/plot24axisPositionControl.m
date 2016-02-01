 set(0,'DefaultAxesFontSize',16)
load('MotorAxisPositionControl25Rotors.mat')
%load('MotorAxisPositionControl50Rotors.mat')
figure(6)
clf
ind = find(T>=30,1,'first');
rg = 1:ind;
set(gcf,'Name', 'Lyapunov Function and Position Plots')
subplot(3,1,1)
plot(T(rg),sum((Y(rg,1:n)-repmat(posDes(1:n)', size(T(rg)))).^2,2),'linewidth',2)
axis tight
a = axis;
axis([a(1),a(2),0,a(4)*1.05]);
ylabel('Error [rad^2]')


subplot(3,1,2:3)
colors = repmat('r',n,1);
colors(posDes>0) = 'b';
plot([0,max(T(rg))],[max(posDes), max(posDes)],'--b','linewidth',1)
hold on
plot(T(rg),Y(rg,1),'linewidth',1,'Color','b')
plot([0,max(T(rg))],[min(posDes), min(posDes)],'--r','linewidth',1)


for i = n:-1:1
    if posDes(i) >0
        myCol = [i/(2*n),i/(2*n),1-i/(8*n)];
    else
        myCol = [1-i/(8*n),i/(2*n),i/(2*n)];
    end
    plot(T(rg),Y(rg,i),'linewidth',1,'Color',myCol)
end
xlabel('Time [s]')
ylabel('Rotor Angle [rad]')

legend('goal','\theta','Location','East')
axis tight
a = axis;
axis([a(1),a(2),a(3)*1.05,a(4)*1.05]);

set(gcf,'papersize',[7,5])
set(gcf,'paperposition',[0,0,7,5])
%print -dpdf '../../pictures/pdf/PositionConverge24state.pdf'
print -dpdf '../../pictures/pdf/PositionConverge25state.pdf'