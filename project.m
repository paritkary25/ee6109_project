% File: project.m
% Author: Zala Mayursinh Meghrajsinh, Yash Udayan Paritkar
% Last Modified: 2024/11/23 22:42:10
% Matlab file for EE6109: EV Powertrain course project

% This Matlab code calculates the gain obtained by adding the supercapacitor in the given vehicle

clc;
clear;


% Data Input


data = readtable('data.xlsx', 'Sheet', 'Sheet1');
v = data.v_m_s_; % velocity data in m/s
gtheta = data.slope; % slope data in radian


% Vehicle Mechanical Parameters


gr = 2; % gear ratio
r = 0.3; % radius of wheel in m
vbase = 40*5/18; % base speed of vehicle in m/s
wbase = gr*vbase/r; % base speed of motor in rad/s
Tmax = 30; % motor max torque in Nm
Pmax = Tmax*wbase; %w


% Vehicle Electrical Parameters


spcap = 10000; % specific power of super capacitor in W/Kg
spbatt = 250; % specific power of battery in W/Kg
spmotor = 1250; % specific power of motor in W/Kg

secap = 5; % specific energy of super capacitor in Wh/Kg
sebatt = 200; % specific energy of battery in Wh/Kg

scostcap = 1600; % cost of super capacitor per Kg
scostbatt = 40;

mcap = 0.2; % mass of super capacitor
mbatt = 6; % mass of battery

pmaxcap = spcap*mcap; % maximum power output of super capacitor
pmaxbatt = spbatt*mbatt; % maximum power output of battery
pmaxregcap = pmaxcap; % maximum power input of super capacitor
pmaxregbatt = 100; % maximum power input of battery

emaxcap = secap*mcap; % maximum energy capacity of super capacitor
emaxbatt = sebatt*mbatt; % maximum energy capacity of battery

costcap = emaxcap*scostcap; % cost of capacitor
costdcdcconv = 1500;
costbatt = emaxbatt*scostbatt;
costveh = 40000;

battlife0 = 5; % battery life in years
battlife1 = 0; % will be determined after calculations

% Mass Calculation


mm = Pmax/spmotor; % mass of motor in Kg
mstr = 14; % mass of structure in Kg
mr = 70; % mass of rider in Kg
mw = 2; % mass of wheel kg
mv = mm + mstr + mcap + mbatt + mr + 2*mw; % total mass in kg
j = 2*(1/2)*mw*r*r; % moment of inertia


% Vehicle Dynamics Parameters


g = 9.8; % gravitational accelaration in m/s*s
urr = 0.004; % friction coefficient of the road
p = 1.225; % air density
A = 0.5; % vehicle front area in m*m
cd = 0.8; % air drag coeeficient
vm = 0; % rest air velocity in m/s

avmax = ((gr*Tmax/r) - (urr*mv*g))/(mv+(j/(r*r))); % acceleration max
vmean = mean(v);
vtemp = 74*5/18;
temp1 = urr*mv*g + 0.5*p*A*cd*((vtemp - vm)^2);
temp2 = Pmax/vtemp;
temp3 = temp2-temp1;


% plotting and time step option


dstep = 1; % time step
dmax = 1300; % time max
acc(dmax) = zeros;
torque(dmax) = zeros;
torquemotor(dmax) = zeros;
power(dmax) = zeros;
powercap(dmax) = zeros;
powerbatt(dmax) = zeros;
time(dmax) = ones;
ep1batt(dmax) = zeros;
ep1cap(dmax) = zeros;
ep2batt(dmax) = zeros;
ep2cap(dmax) = zeros;
ep1batt(1) = emaxbatt;
ep2batt(1) = emaxbatt;

% Without Super Capacitor


ecap = 0;
ebatt = emaxbatt;
emaxcap = 0;

for i = 2:dstep:dmax
    
    fav = p*A*cd*((v(i,1)-vm)^2)/2; % aerodynamic drag
    frr = urr*mv*g*cos(gtheta(i,1)); % rolling resistance
    fhc = mv*g*sin(gtheta(i,1)); % hill force
    acc(i) = (v(i,1) - v(i-1,1))/dstep; % acceleration
    torque(i) = (((mv+(j/(r*r)))*acc(i)) + frr + fav + fhc)*r/gr; % required torque
    
    % motor torque calculation

    if(torque(i) > 0) % while accelerating
        torquemotor(i) = min(torque(i), tmotor(Tmax, wbase, gr*v(i)/r));
    else % while braking
        torquemotor(i) = max(torque(i), -1*tmotor(Tmax, wbase, gr*v(i)/r));
    end
    
    time(i) = i;
    power(i) = torquemotor(i)*gr*v(i)/r; % motor power
    
    if(power(i) > 0) % power consuption
        if(ecap > power(i)*dstep/3600)
            ecap = ecap - power(i)*dstep/3600;
            powercap(i) = power(i);
            powerbatt(i) = 0;
        elseif(ebatt > power(i)*dstep/3600)
            ebatt = ebatt - power(i)*dstep/3600;
            powerbatt(i) = power(i);
            powercap(i) = 0;
        end
    elseif(power(i) < 0) % power regenration
        if(ecap < emaxcap)
            if(power(i) > -1*pmaxregcap)
                ecap = ecap - power(i)*dstep/3600;
                powercap(i) = power(i);
                powerbatt(i) = 0;
            elseif(power(i) < -1*pmaxregcap)
                ecap = ecap + pmaxregcap*dstep/3600;
                powercap(i) = -1*pmaxregcap;
                if(ebatt < emaxbatt)
                    ebatt = ebatt - max(power(i)-pmaxregcap, -1*pmaxregbatt)*dstep/3600;
                    powerbatt(i) = max(power(i)-pmaxregcap, -1*pmaxregbatt);
                end
            end
        elseif(ebatt < emaxbatt)
            ebatt = ebatt - max(power(i), -1*pmaxregbatt)*dstep/3600;
            powerbatt(i) = max(power(i), -1*pmaxregbatt);
        end
    end
    
    ep1cap(i) = ecap;
    ep1batt(i) = ebatt;

end


% With Super Capacitor


emaxcap = secap*mcap;
ebatt = emaxbatt;
ecap = 0;

for i = 2:dstep:dmax
    
    fav = p*A*cd*((v(i,1)-vm)^2)/2; % aerodynamic drag
    frr = urr*mv*g*cos(gtheta(i,1)); % rolling resistance
    fhc = mv*g*sin(gtheta(i,1)); % hill force
    acc(i) = (v(i,1) - v(i-1,1))/dstep; % acceleration
    torque(i) = (((mv+(j/(r*r)))*acc(i)) + frr + fav + fhc)*r/gr; % required torque
    
    % motor torque calculation

    if(torque(i) > 0) % while accelerating
        torquemotor(i) = min(torque(i), tmotor(Tmax, wbase, gr*v(i)/r));
    else % while braking
        torquemotor(i) = max(torque(i), -1*tmotor(Tmax, wbase, gr*v(i)/r));
    end
    
    time(i) = i;
    power(i) = torquemotor(i)*gr*v(i)/r; % motor power
    
    if(power(i) > 0) % power consuption
        if(ecap > power(i)*dstep/3600)
            ecap = ecap - power(i)*dstep/3600;
            powercap(i) = power(i);
            powerbatt(i) = 0;
        elseif(ebatt > power(i)*dstep/3600)
            ebatt = ebatt - power(i)*dstep/3600;
            powerbatt(i) = power(i);
            powercap(i) = 0;
        end
    elseif(power(i) < 0) % power regenration
        if(ecap < emaxcap + power(i)*dstep/3600)
            if(power(i) > -1*pmaxregcap)
                ecap = ecap - power(i)*dstep/3600;
                powercap(i) = power(i);
                powerbatt(i) = 0;
            elseif(power(i) < -1*pmaxregcap)
                ecap = ecap + pmaxregcap*dstep/3600;
                powercap(i) = -1*pmaxregcap;
                if(ebatt < emaxbatt)
                    ebatt = ebatt - max(power(i)-pmaxregcap, -1*pmaxregbatt)*dstep/3600;
                    powerbatt(i) = max(power(i)-pmaxregcap, -1*pmaxregbatt);
                end
            end
        elseif(ebatt < emaxbatt)
            ebatt = ebatt - max(power(i), -1*pmaxregbatt)*dstep/3600;
            powerbatt(i) = max(power(i), -1*pmaxregbatt);
        end
    end
    
    ep2cap(i) = ecap;
    ep2batt(i) = ebatt;

end


% Data plotting


f1 = figure(1);
set(gca,'FontSize',17,'fontWeight','bold');
set(findall(gcf,'type','text'),'FontSize',17,'fontWeight','bold');
hold on;
plot(time, v(1:dmax,1)*18/5, "DisplayName", 'Velocity', "LineWidth", 1.5);
title('Velocity with time');
xlabel('Time (sec)');
ylabel('Velocity (Km/hr)');
axis([0 dmax (min(v)*18/5)-1 (max(v)*18/5)+1]);
legend show;
saveas(f1, 'D:\study\7th_sem\ee_6109\project\Velocity.svg', 'svg');

f2 = figure(2);
set(gca,'FontSize',17,'fontWeight','bold');
set(findall(gcf,'type','text'),'FontSize',17,'fontWeight','bold');
hold on;
plot(time, torque, "DisplayName", 'Vehicle torque', "LineWidth", 1.5);
plot(time, torquemotor, "DisplayName", 'Motor torque', "LineWidth", 1.5);
title('Torque performance');
xlabel('Time (sec)');
ylabel('Torue (Nm)');
axis([0 dmax min(min(torque),min(torquemotor))-1 max(max(torque),max(torquemotor))+1]);
legend show;
saveas(f2, 'D:\study\7th_sem\ee_6109\project\Torque.svg', 'svg');

f3 = figure(3);
set(gca,'FontSize',17,'fontWeight','bold');
set(findall(gcf,'type','text'),'FontSize',17,'fontWeight','bold');
hold on;
plot(time, acc, "DisplayName", 'Accelaration', "LineWidth", 1.5);
title('Acceleration with time');
xlabel('Time (sec)');
ylabel('Acceleration (m/s*s)');
axis([0 dmax min(acc)-1 max(acc)+1]);
legend show;
saveas(f3, 'D:\study\7th_sem\ee_6109\project\Accelaration.svg', 'svg');

f4 = figure(4);
set(gca,'FontSize',17,'fontWeight','bold');
set(findall(gcf,'type','text'),'FontSize',17,'fontWeight','bold');
hold on;
plot(time, power, "DisplayName", 'Total power', "LineWidth", 1.5);
title('Power performance');
xlabel('Time (sec)');
ylabel('Power (w)');
axis([0 dmax min(power)-1 max(power)+1]);
legend show;
saveas(f4, 'D:\study\7th_sem\ee_6109\project\Total_power.svg', 'svg');

f5 = figure(5);
set(gca,'FontSize',17,'fontWeight','bold');
set(findall(gcf,'type','text'),'FontSize',17,'fontWeight','bold');
hold on;
plot(time, powercap, "DisplayName", 'Capacitor', "LineWidth", 1.5);
plot(time, powerbatt, "DisplayName", 'Battery', "LineWidth", 1.5);
title('Power performance');
xlabel('Time (sec)');
ylabel('Power (w)');
axis([0 dmax min(min(powercap),min(powerbatt))-1 max(max(powercap),max(powerbatt))+1]);
legend show;
saveas(f5, 'D:\study\7th_sem\ee_6109\project\Power_compare.svg', 'svg');

f6 = figure(6);
set(gca,'FontSize',17,'fontWeight','bold');
set(findall(gcf,'type','text'),'FontSize',17,'fontWeight','bold');
hold on;
plot(time, ep1cap*100/emaxcap, "DisplayName", 'Without SC', "LineWidth", 1.5);
plot(time, ep2cap*100/emaxcap, "DisplayName", 'With SC', "LineWidth", 1.5);
title('Capacitor performance');
xlabel('Time (sec)');
ylabel('Capacitor energy (%)');
axis([0 dmax 0 100]);
legend show;
saveas(f6, 'D:\study\7th_sem\ee_6109\project\Capacitor_energy.svg', 'svg');

f7 = figure(7);
set(gca,'FontSize',17,'fontWeight','bold');
set(findall(gcf,'type','text'),'FontSize',17,'fontWeight','bold');
hold on;
plot(time, ep1batt*100/emaxbatt, "DisplayName", 'Without SC', "LineWidth", 1.5);
plot(time, ep2batt*100/emaxbatt, "DisplayName", 'With SC', "LineWidth", 1.5);
title('Battery performance');
xlabel('Time (sec)');
ylabel('Battery energy (%)');
axis([0 dmax 85 100]);
legend show;
saveas(f7, 'D:\study\7th_sem\ee_6109\project\Battery_energy.svg', 'svg');


% Total Driving Time and Improvement In Total Time and range


time0 = emaxbatt*dmax/((emaxbatt-ep1batt(dmax))*3600);
time1 = emaxbatt*dmax/((emaxbatt-ep2batt(dmax))*3600);

range0 = vmean*time0*3600/1000;
range1 = vmean*time1*3600/1000;

eperkm0 = emaxbatt/range0;
eperkm1 = emaxbatt/range1;

battlife1 = time1*battlife0/time0;

cost0 = costbatt + costveh;
cost1 = costbatt + costveh + costcap + costdcdcconv;

minutepercost0 = 60*24*365*battlife0/cost0;
minutepercost1 = 60*24*365*battlife1/cost1;

costperextralife = (cost1 - cost0)/(battlife1 - battlife0);

improvement_time = time1/time0;
improvement_range = range1/range0;
improvement_batterylife = battlife1/battlife0;
improvement_energy = eperkm0/eperkm1;

display(improvement_energy*100 - 100);


% Motor max torque function


function y = tmotor(Tmax, wbase, w)
   if(w < wbase)
        y = Tmax;
   else
        y = Tmax*wbase/w;
   end
end