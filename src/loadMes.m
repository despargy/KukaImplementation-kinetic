yD = load('Collector/posCollected_cut.txt');
yD = yD';

m = load('Messures/robotJointPositions.txt');
j = 0 ;
for i = 1:8010
    for d = 1:7
        j = j + 1;
        mes(d,i) = m(j);
    end
end

ys(1,:) = load('CHECK/y_smoothed0.log');
ys(2,:) = load('CHECK/y_smoothed1.log');
ys(3,:) = load('CHECK/y_smoothed2.log');
ys(4,:) = load('CHECK/y_smoothed3.log');
ys(5,:) = load('CHECK/y_smoothed4.log');
ys(6,:) = load('CHECK/y_smoothed5.log');
ys(7,:) = load('CHECK/y_smoothed6.log');

figure; hold on ; plot(ys(1,:)) ; plot(yD(1,:)); 
figure; hold on ; plot(ys(2,:)) ; plot(yD(2,:)); 
figure; hold on ; plot(ys(3,:)) ; plot(yD(3,:)); 
figure; hold on ; plot(ys(4,:)) ; plot(yD(4,:)); 
figure; hold on ; plot(ys(5,:)) ; plot(yD(5,:)); 
figure; hold on ; plot(ys(6,:)) ; plot(yD(6,:)); 
figure; hold on ; plot(ys(7,:)) ; plot(yD(7,:)); 

% Reverse
for d = 1:7
    for i = 1:8010
        RyD(d,i) = yD(d, 8010 - i + 1);
    end
end
Rm = load('Messures/RrobotJointPositions.txt');
j = 0 ;
for i = 1:8010
    for d = 1:7
        j = j + 1;
        Rmes(d,i) = Rm(j);
    end
end

Rys0 = load('CHECK/Ry_smoothed0.log');
Rys1 = load('CHECK/Ry_smoothed1.log');
Rys2 = load('CHECK/Ry_smoothed2.log');
Rys3 = load('CHECK/Ry_smoothed3.log');
Rys4 = load('CHECK/Ry_smoothed4.log');
Rys5 = load('CHECK/Ry_smoothed5.log');
Rys6 = load('CHECK/Ry_smoothed6.log');

figure; hold on ; plot(Rys0) ; plot(RyD(1,:))
figure; hold on ; plot(Rys1) ; plot(RyD(2,:))
figure; hold on ; plot(Rys2) ; plot(RyD(3,:))
figure; hold on ; plot(Rys3) ; plot(RyD(4,:))
figure; hold on ; plot(Rys4) ; plot(RyD(5,:))
%% 
figure; hold on ; plot(Rys5) ; plot(RyD(6,:))
figure; hold on ; plot(Rys6) ; plot(RyD(7,:))




