function GUI_Variables = command(GUI_Variables, msg, data, handles)
%This function Figures out what non-data
%message the bluetooth sent

% i.e. it is used when matlab receive from arduino a message non-data
switch(msg)
  case '?'% It means it is data message to plot and update signals
    RLCount = GUI_Variables.RLCount;
    LLCount = GUI_Variables.LLCount;
    
    GUI_Variables.RLTRQ(RLCount) = data(1);                 %Gets the new Torque Value and Stores it
    GUI_Variables.RLFSR(RLCount) = data(2); %state
    GUI_Variables.RLSET(RLCount) = data(3); %save the torque set point
    GUI_Variables.RLVOLT(RLCount) = data(4);
    GUI_Variables.RLVOLT_H(RLCount) = data(5);
    GUI_Variables.SIG1(RLCount) = data(11);
    GUI_Variables.SIG3(RLCount) = data(13);
    GUI_Variables.SIG4(RLCount) = data(14);%data(14)/100;
    GUI_Variables.BASER(RLCount)=GUI_Variables.baser;

    GUI_Variables.R_BAL_DYN_HEEL(RLCount)=GUI_Variables.R_Bal_dyn_Heel;
    GUI_Variables.R_BAL_STEADY_HEEL(RLCount)=GUI_Variables.R_Bal_steady_Heel;
    GUI_Variables.R_BAL_DYN_TOE(RLCount)=GUI_Variables.R_Bal_dyn_Toe;
    GUI_Variables.R_BAL_STEADY_TOE(RLCount)=GUI_Variables.R_Bal_steady_Toe;
    GUI_Variables.BASEL_BIOFB(RLCount)=GUI_Variables.basel_biofb;
           
    GUI_Variables.LLTRQ(LLCount) = data(6);            %Gets the new Torque Value and stores it
    GUI_Variables.LLFSR(LLCount) = data(7);
    GUI_Variables.LLSET(LLCount) = data(8); %New to save also the set point
    GUI_Variables.LLVOLT(LLCount) = data(9);
    GUI_Variables.LLVOLT_H(LLCount) = data(10);
    GUI_Variables.SIG2(LLCount) = data(12);

    GUI_Variables.BASEL(LLCount)=GUI_Variables.basel;

    GUI_Variables.L_BAL_DYN_HEEL(LLCount)=GUI_Variables.L_Bal_dyn_Heel;
    GUI_Variables.L_BAL_STEADY_HEEL(LLCount)=GUI_Variables.L_Bal_steady_Heel;
    GUI_Variables.L_BAL_DYN_TOE(LLCount)=GUI_Variables.L_Bal_dyn_Toe;
    GUI_Variables.L_BAL_STEADY_TOE(LLCount)=GUI_Variables.L_Bal_steady_Toe;

    GUI_Variables.LLCount = LLCount + 1;
    GUI_Variables.RLCount = RLCount + 1;
    if(data(2)==9)||(data(7)==9)
        disp("Torque value problem    Trq > 25Nm");
        set(handles.statusText,'String','Problem Trq Ctrl, Trq > 25 Nm');

    end
  case '`'
    KF_LL = data(1);                                          %Gets the Current Arduino Torque Setpoint
    set(handles.L_Check_KF_Text,'String',KF_LL);
    disp("Left Current KF ");
    disp(KF_LL);

  case '~'
    KF_RL = data(1);                                            %Gets the Current Arduino Torque Setpoint
    set(handles.R_Check_KF_Text,'String',KF_RL);
    disp("Right Current KF ");
    disp(KF_RL);
  case 'D'
    Setpoint_LL = data(1);
    Setpoint_Dorsi_LL = data(2);
    update_setpoint(Setpoint_LL, handles.L_Setpoint_Text,handles.L_Setpoint_Edit);
    update_setpoint(Setpoint_Dorsi_LL, handles.L_Setpoint_Dorsi_Text,handles.L_Setpoint_Dorsi_Edit);
  case 'd'
    Setpoint_RL = data(1);
    Setpoint_Dorsi_RL = data(2);
    update_value_handles(Setpoint_RL, handles.R_Setpoint_Text,handles.R_Setpoint_Edit);
    update_value_handles(Setpoint_Dorsi_RL, handles.R_Setpoint_Dorsi_Text,handles.R_Setpoint_Dorsi_Edit);
  case 'K'
    lkp = data(1);
    lkd = data(2);
    lki = data(3);
    update_value_handles(lkp, handles.L_Kp_text, handles.L_Kp_Edit);
    update_value_handles(lkd, handles.L_Kd_text, handles.L_Kd_Edit);
    update_value_handles(lki, handles.L_Ki_text, handles.L_Ki_Edit);
  case 'k'
    rkp = data(1);
    rkd = data(2);
    rki = data(3);
    update_value_handles(rkp, handles.R_Kp_text, handles.R_Kp_Edit);
    update_value_handles(rkd, handles.R_Kd_Text, handles.R_Kd_Edit);
    update_value_handles(rki, handles.R_Ki_Text, handles.R_Ki_Edit);
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
    val_biofb=strcmp(get(handles.Activate_BioFeedback_Text,'String'),'On');
    
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
  case '<'
      
    mem=GUI_Variables.MEM;
    check_torque = data(1);
    check_FSR = data(2);
    check_EXP = data(3);
    mem = check_memory(check_torque, handles.axes10,mem,2);
    mem = check_memory(check_FSR, handles.axes8,mem,1);
    mem = check_memory(check_EXP, handles.EXP_Params_axes,mem,3);
    GUI_Variables.MEM = mem;
  case 'U'
    version = round(data(1));
    major = mod(floor(version/100),10);
    minor = mod(floor(version/10),10);
    sub_minor = mod(floor(version/1),10);
    str = sprintf("Reported code version %d.%d.%d", major, minor, sub_minor);
    set(handles.statusText,'String',str);
    
  case 'z'
    set(handles.Motor_Error,'value',data(1));

  case 'N'
    mem=GUI_Variables.MEM;
    check1 = data(1);
    check2 = data(2);
    check3 = data(3);
    if(check1 == 0 && check2 == 1 && check3 == 2)
        set(handles.statusText,'String',"Working as Expected!");
        set(handles.flag_bluetooth,'Color',[0 1 0]);

        set_memory_color(handles.axes8, mem(1))
        set_memory_color(handles.axes10, mem(2))
        set_memory_color(handles.EXP_Params_axes, mem(3))

        valBT=1;

    else
        set(handles.statusText,'String',"A problem Occured and Bt has been closed!");
        set(handles.flag_bluetooth,'Color',[1 0 0]);
        set(handles.axes8,'Color',[0 0 0])
        set(handles.axes10,'Color',[0 0 0])
        set(handles.EXP_Params_axes,'Color',[0 0 0])
        valBT=0;
        fclose(bt);
    end
  otherwise
    %Do nothing
end

function update_value_handles(new_value, text_handle, edit_handle)
    old_setpoint = get(text_handle,'String');
    current_edit = get(edit_handle,'String');
    set(text_handle,'String',new_value);
    if (strcmp(current_edit,"NaN")) || (~isempty(old_setpoint) && strcmp(old_setpoint,current_edit))
        set(edit_handle,'String',new_value);
    end
    
function mem = check_memory(check, axis, mem, mem_index)
    mem(mem_index) = check;
    set_memory_color(axis, check)


function set_memory_color(axis, value)
    if value==0
        set(axis,'Color',[1 0 0])
    elseif value==1
        set(axis,'Color',[0 1 0])
    else
        set(axis,'Color',[0 0 1])
    end