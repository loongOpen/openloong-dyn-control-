clear variables; close all
dataRec=load('datalog.log');
simTime=dataRec(:,1:1);
motor_pos_des=dataRec(:,2:32);
motor_pos_cur=dataRec(:,33:63);
motor_vel_cur=dataRec(:,64:94);
motor_tor_des=dataRec(:,95:125);
motor_tor_out=dataRec(:,126:156);
rpyVal=dataRec(:,157:159);
gpsVal=dataRec(:,160:162);
fe_l_pos_L_des=dataRec(:,163:165);
fe_r_pos_L_des=dataRec(:,166:168);
fe_l_pos_W=dataRec(:,169:171);
fe_r_pos_W=dataRec(:,172:174);
Ufe=dataRec(:,175:186);

[rowt,colt] = size(simTime);
ranget = 1:rowt;

figure("Name","motor pos");
for i = 1:3
    subplot(3,6,i);
    plot(simTime(ranget,1), motor_pos_cur(ranget,i+16)*180/pi,'b-',...
         simTime(ranget,1), motor_pos_des(ranget,i+16)*180/pi,'r-')
end
for i = 1:12
    subplot(3,6,i+6);
    plot(simTime(ranget,1), motor_pos_cur(ranget,i+19)*180/pi,'b-',...
         simTime(ranget,1), motor_pos_des(ranget,i+19)*180/pi,'r-')
end

figure("Name","motor tor out");
for i = 1:3
    subplot(3,6,i);
    plot(simTime(ranget,1), motor_tor_out(ranget,i+16),'b-',...
        simTime(ranget,1), motor_tor_des(ranget,i+16),'r-');
end
for i = 1:12
    subplot(3,6,i+6);
    plot(simTime(ranget,1), motor_tor_out(ranget,i+19),'b-',...
        simTime(ranget,1), motor_tor_des(ranget,i+19),'r-');
end

figure("Name","Ufe");
for i = 1:12
    subplot(4,3,i);
    plot(simTime(ranget,1), Ufe(ranget,i)); 
    grid on;
end

figure("Name","fe");
for i = 1:3
    subplot(2,3,i);
    plot(simTime(ranget,1), fe_l_pos_W(ranget,i));
    grid on;
    subplot(2,3,i+3);
    plot(simTime(ranget,1), fe_l_pos_W(ranget,i));
    grid on;
end

