
close all
clc
str=input('Name of the file: ','s');
fprintf(['Filename is: ',str,'\n']);

fprintf(['Importing data \n']);

MAT=importfile_last(str);

fprintf(['Data Imported \n']);

str2=input('Name of the folder to save results: ','s');
if strcmp(str2,'')
    str2=str;
else
    str=str2;
end

time=MAT(:,1);
RLTRQ=MAT(:,2);
RLSTATE=MAT(:,3);
RLSET=MAT(:,4);
RLVOLT_PEAK=MAT(:,5);
RLVOLT_ALL=MAT(:,6);

LLTRQ=MAT(:,7);
LLSTATE=MAT(:,8);
LLSET=MAT(:,9);
LLVOLT_PEAK=MAT(:,10);
LLVOLT_ALL=MAT(:,11);

TRIG=MAT(:,12);
LL_BASE=MAT(:,13);
RL_BASE=MAT(:,14);

LSPEED=MAT(:,15);
RSPEED=MAT(:,16);

LLVOLT_HEEL=MAT(:,16);
RLVOLT_HEEL=MAT(:,17);
LL_RATIO=MAT(:,18);
RL_RATIO=MAT(:,19);

str_fld=[str,'_fld'];
mkdir(str_fld);

figure('Name','Right')
subplot(3,1,1)
plot(time,[RLTRQ,RLSET],'LineWidth',2)
title(' Right Ankle Torque')
ylabel('Nm')
legend('Meas','Des')
set(gca,'FontSize',15)
grid on


subplot(3,1,2)
plot(time,RLSTATE,'LineWidth',2)
title('Right Step State')
xlabel('Time')
ylabel('#')
grid on
set(gca,'FontSize',15)


subplot(3,1,3)
plot(time,TRIG,'LineWidth',2)
title('Right Trig State')
xlabel('Time')
ylabel('#')
grid on
set(gca,'FontSize',15)
saveas(gcf,[str_fld,'\Right'],'fig')

% % % 
% % % figure('Name','Left')
% % % subplot(3,1,1)
% % % plot(time,[LLTRQ,LLSET],'LineWidth',2)
% % % title(' Left Ankle Torque')
% % % ylabel('Nm')
% % % legend('Meas','Des')
% % % set(gca,'FontSize',15)
% % % grid on
% % % 
% % % 
% % % subplot(3,1,2)
% % % plot(time,LLSTATE,'LineWidth',2)
% % % title('Left Step State')
% % % xlabel('Time')
% % % ylabel('#')
% % % grid on
% % % set(gca,'FontSize',15)
% % % 
% % % 
% % % subplot(3,1,3)
% % % plot(time,TRIG,'LineWidth',2)
% % % title('Left Trig State')
% % % xlabel('Time')
% % % ylabel('#')
% % % grid on
% % % set(gca,'FontSize',15)
% % % saveas(gcf,[str_fld,'\Left'],'fig')
% % % 
% % % if exist([str,'_Bias'])
% % % load([str,'_Bias']);
% % % end
% % % 
% % % figure
% % % subplot(2,1,1)
% % % plot(time,LLSTATE,'LineWidth',2)
% % % title('Left Step State')
% % % xlabel('Time')
% % % ylabel('#')
% % % grid on
% % % set(gca,'FontSize',15)
% % % 
% % % subplot(2,1,2)
% % % plot(time,LLVOLT_PEAK,'LineWidth',2)
% % % hold on
% % % plot(time,LLVOLT_ALL,'LineWidth',2)
% % % legend('Toe','Heel')
% % % if exist([str,'_Bias'])
% % % hold on
% % % plot(time,time*0+L_fsr_short)
% % % end
% % % title('Left Volt')
% % % xlabel('Time')
% % % ylabel('#')
% % % grid on
% % % set(gca,'FontSize',15)
% % % saveas(gcf,[str_fld,'\Left_Volt'],'fig')
% % % 
% % % figure
% % % subplot(2,1,1)
% % % plot(time,RLSTATE,'LineWidth',2)
% % % title('Right Step State')
% % % xlabel('Time')
% % % ylabel('#')
% % % grid on
% % % set(gca,'FontSize',15)
% % % 
% % % subplot(2,1,2)
% % % plot(time,RLVOLT_PEAK,'LineWidth',2)
% % % hold on
% % % plot(time,RLVOLT_ALL,'LineWidth',2)
% % % legend('Toe','Heel')
% % % if exist([str,'_Bias'])
% % % hold on
% % % plot(time,time*0+R_fsr_short)
% % % end
% % % title('Right Volt')
% % % xlabel('Time')
% % % ylabel('#')
% % % grid on
% % % set(gca,'FontSize',15)
% % % saveas(gcf,[str_fld,'\Right_Volt'],'fig')
% % % 
% % % 
% % % % plot Torque Vs Volt
% % % 
% % % figure('Name','Left_TV')
% % % subplot(2,1,1)
% % % plot(time,[LLTRQ,LLSET],'LineWidth',2)
% % % title(' Left Ankle Torque')
% % % ylabel('Nm')
% % % legend('Meas','Des')
% % % set(gca,'FontSize',15)
% % % grid on
% % % 
% % % subplot(2,1,2)
% % % plot(time,LLVOLT_PEAK,'LineWidth',2)
% % % hold on
% % % plot(time,LLVOLT_ALL,'LineWidth',2)
% % % legend('Toe','Heel')
% % % title('Left Volt')
% % % xlabel('Time')
% % % ylabel('#')
% % % grid on
% % % set(gca,'FontSize',15)
% % % saveas(gcf,[str_fld,'\Right_TV'],'fig')
% % % 
% % % 
% % % 
% % % figure('Name','Right_TV')
% % % subplot(2,1,1)
% % % plot(time,[RLTRQ,RLSET],'LineWidth',2)
% % % title(' Right Ankle Torque')
% % % ylabel('Nm')
% % % legend('Meas','Des')
% % % set(gca,'FontSize',15)
% % % grid on
% % % 
% % % subplot(2,1,2)
% % % plot(time,RLVOLT_PEAK,'LineWidth',2)
% % % hold on
% % % plot(time,RLVOLT_ALL,'LineWidth',2)
% % % legend('Toe','Heel')
% % % title('Right Volt')
% % % xlabel('Time')
% % % ylabel('#')
% % % grid on
% % % set(gca,'FontSize',15)
% % % saveas(gcf,[str_fld,'\Left_TV'],'fig')
% % % 
% % % 
% % % %%%%
% % % figure('Name','Left_Right_COP')
% % % % subplot(2,1,1)
% % % plot(time,[LCOP,RCOP],'LineWidth',2)
% % % title(' L R COP Evolution')
% % % ylabel('Nm')
% % % legend('LCOP','RCOP')
% % % set(gca,'FontSize',15)
% % % grid on
% % % % 
% % % % subplot(2,1,2)
% % % % plot(time,LLVOLT,'LineWidth',2)
% % % % title('Left Volt')
% % % % xlabel('Time')
% % % % ylabel('#')
% % % % grid on
% % % % set(gca,'FontSize',15)
% % % saveas(gcf,[str_fld,'\LR_COPs'],'fig')
% % % 
% % % fprintf(['Complete \n']);