function [] = command(msg, data, handles)
global GUI_Variables;
%This function Figures out what non-data
%message the bluetooth sent

% i.e. it is used when matlab receive from arduino a message non-data
switch(msg)
  case '`'
    KF_LL = data(1);                                          %Gets the Current Arduino Torque Setpoint
    set(handles.L_Check_KF_Text,'String',KF_LL);
    disp("Right Current KF ");
    disp(KF_LL);

  case '~'
    KF_RL = data(1);                                            %Gets the Current Arduino Torque Setpoint
    set(handles.R_Check_KF_Text,'String',KF_RL);
    disp("Right Current KF ");
    disp(KF_RL);
  case 'D'
    Setpoint_LL = data(1);
    set(handles.L_Setpoint_Text,'String',Setpoint_LL);
    Setpoint_Dorsi_LL = data(2);
    set(handles.L_Setpoint_Dorsi_Text,'String',Setpoint_Dorsi_LL);
  case 'd'
    Setpoint_RL = data(1);
    set(handles.R_Setpoint_Text,'String',Setpoint_RL);
    Setpoint_Dorsi_RL = data(2);
    set(handles.R_Setpoint_Dorsi_Text,'String',Setpoint_Dorsi_RL);
  case 'K'
    lkp = data(1);
    lkd = data(2);
    lki = data(3);
    set(handles.L_Kp_text,'String',lkp);
    set(handles.L_Kd_text,'String',lkd);
    set(handles.L_Ki_text,'String',lki);
  case 'k'
    rkp = data(1);
    rkd = data(2);
    rki = data(3);
    set(handles.R_Kp_text,'String',rkp);
    set(handles.R_Kd_Text,'String',rkd);
    set(handles.R_Ki_Text,'String',rki);
  case 'Q'
    FSR_thresh_LL = data(1);
    set(handles.L_Check_FSR_Text,'String',FSR_thresh_LL);
    disp("Left Current FSR th ");
    disp(FSR_thresh_LL);
  case 'q'
    Curr_TH_R = data(1);
    set(handles.R_Check_FSR_Text,'String',Curr_TH_R);
    disp("Right Current FSR th ");
    disp(Curr_TH_R);
  case '('
    N1 = data(1);
    N2 = data(2);
    N3 = data(3);
    set(handles.N1_Text,'String',N1);
    set(handles.N2_Text,'String',N2);
    set(handles.N3_Text,'String',N3);
  case '}' %Send Left Gain to Arduino
    L_Gain = data(1);
    set(handles.L_Check_Gain_Text,'String',L_Gain);
  case ']' % Send Right Gain to Arduino
    R_Gain = data(1);
    set(handles.R_Check_Gain_Text,'String',R_Gain);
  case 'B'
    val=strcmp(get(handles.Balance_Text,'String'),'On');
    get(handles.Activate_BioFeedback_Text,'String')
    val_biofb=strcmp(get(handles.Activate_BioFeedback_Text,'String'),'On')
    
    if (val_biofb==1)
        disp('biofeedback baseline');
        GUI_Variables.basel_biofb=data(1);
        disp(GUI_Variables.basel_biofb)
    elseif (val==1)
        disp('balance baseline');
        GUI_Variables.L_Bal_steady_Toe= data(1);
        GUI_Variables.L_Bal_steady_Heel= data(2);
        GUI_Variables.R_Bal_steady_Toe= data(3);
        GUI_Variables.R_Bal_steady_Heel= data(4);
        
        GUI_Variables.L_Bal_dyn_Toe= data(5);
        GUI_Variables.L_Bal_dyn_Heel= data(6);
        GUI_Variables.R_Bal_dyn_Toe= data(7);
        GUI_Variables.R_Bal_dyn_Heel= data(8);
    elseif(val==0)
        GUI_Variables.basel= data(1);
        GUI_Variables.baser= data(2);
    disp('command');
    disp(GUI_Variables.basel);
    disp(GUI_Variables.baser);

    end
  case 'V'
    steady_val = str2double(message((indexes(1)+1):(indexes(2)-1)));
    set(handles.Steady_Text,'String',steady_val);
    disp(["Check Steady Val ",num2str(steady_val)]);
  case 'A'
    dyn_val = str2double(message((indexes(1)+1):(indexes(2)-1)));
    set(handles.Dyn_Text,'String',dyn_val);
    disp(["Check Dyn Val ",num2str(dyn_val)]);
  otherwise
    %Do nothing
end
