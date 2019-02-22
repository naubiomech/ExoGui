function [RLCount,LLCount] = Receive_Data_Message(RLCount,LLCount,hObject, eventdata, handles)
global GUI_Variables;
[msg, Data] = get_message();
if(msg == '?') % It means it is data message to plot and update signals
    GUI_Variables.RLTRQ(RLCount) = Data(1);                 %Gets the new Torque Value and Stores it
    GUI_Variables.RLFSR(RLCount) = Data(2); %state
    GUI_Variables.RLSET(RLCount) = Data(3); %save the torque set point
    GUI_Variables.RLVOLT(RLCount) = Data(4);
    GUI_Variables.RLVOLT_H(RLCount) = Data(5);
    GUI_Variables.SIG1(RLCount) = Data(11);
    GUI_Variables.SIG3(RLCount) = Data(13);
    GUI_Variables.SIG4(RLCount) = Data(14);%Data(14)/100;
    GUI_Variables.BASER(RLCount)=GUI_Variables.baser;

    GUI_Variables.R_BAL_DYN_HEEL(RLCount)=GUI_Variables.R_Bal_dyn_Heel;
    GUI_Variables.R_BAL_STEADY_HEEL(RLCount)=GUI_Variables.R_Bal_steady_Heel;
    GUI_Variables.R_BAL_DYN_TOE(RLCount)=GUI_Variables.R_Bal_dyn_Toe;
    GUI_Variables.R_BAL_STEADY_TOE(RLCount)=GUI_Variables.R_Bal_steady_Toe;
    GUI_Variables.BASEL_BIOFB(RLCount)=GUI_Variables.basel_biofb;

    RLCount = RLCount + 1;                                         %Increments kneeCount                                          %Checks if Ankle Arduino Sent a new Torque Value
    GUI_Variables.RLCount = RLCount;
    GUI_Variables.LLTRQ(LLCount) = Data(6);            %Gets the new Torque Value and stores it
    GUI_Variables.LLFSR(LLCount) = Data(7);
    GUI_Variables.LLSET(LLCount) = Data(8); %New to save also the set point
    GUI_Variables.LLVOLT(LLCount) = Data(9);
    GUI_Variables.LLVOLT_H(LLCount) = Data(10);
    GUI_Variables.SIG2(LLCount) = Data(12);

    GUI_Variables.BASEL(LLCount)=GUI_Variables.basel;

    GUI_Variables.L_BAL_DYN_HEEL(LLCount)=GUI_Variables.L_Bal_dyn_Heel;
    GUI_Variables.L_BAL_STEADY_HEEL(LLCount)=GUI_Variables.L_Bal_steady_Heel;
    GUI_Variables.L_BAL_DYN_TOE(LLCount)=GUI_Variables.L_Bal_dyn_Toe;
    GUI_Variables.L_BAL_STEADY_TOE(LLCount)=GUI_Variables.L_Bal_steady_Toe;



    LLCount = LLCount + 1;
    GUI_Variables.LLCount = LLCount;
    if(Data(2)==9)||(Data(7)==9)
        disp("Torque value problem    Trq > 25Nm");
        set(handles.statusText,'String','Problem Trq Ctrl, Trq > 25 Nm');

    end

else % it is a non data message
    command(msg, Data, handles);
end
