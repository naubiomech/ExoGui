function [RLCount,LLCount] = Receive_Data_Message(RLCount,LLCount,hObject, eventdata, handles)
    global GUI_Variables;
    [msg, Data] = get_message();
    if(msg == 63) % It means it is data message to plot and update signals
        GUI_Variables.RLTorque(RLCount) = Data(1);                 %Gets the new Torque Value and Stores it
        GUI_Variables.RLFSR(RLCount) = Data(2);
        GUI_Variables.RLSET(RLCount) = Data(3); %New to save also the set point
        GUI_Variables.RLVOLT(RLCount) = Data(4);
        GUI_Variables.RLVOLT_H(RLCount) = Data(5);
        GUI_Variables.SIG1(RLCount) = Data(11);
        GUI_Variables.SIG3(RLCount) = Data(13);
        GUI_Variables.SIG4(RLCount) = Data(14);
        GUI_Variables.BASER(RLCount)=GUI_Variables.baser;
        RLCount = RLCount + 1;                                         %Increments kneeCount                                          %Checks if Ankle Arduino Sent a new Torque Value
        GUI_Variables.RLCount = RLCount;
        GUI_Variables.LLTorque(LLCount) = Data(6);              %Gets the new Torque Value and stores it
        GUI_Variables.LLFSR(LLCount) = Data(7);
        GUI_Variables.LLSET(LLCount) = Data(8); %New to save also the set point
        GUI_Variables.LLVOLT(LLCount) = Data(9);
        GUI_Variables.LLVOLT_H(LLCount) = Data(10);
        GUI_Variables.SIG2(LLCount) = Data(12);

        GUI_Variables.BASEL(LLCount)=GUI_Variables.basel;

        LLCount = LLCount + 1;
        GUI_Variables.LLCount = LLCount;
        %                   pause(.000000001);                                               %Pauses to give time for the user to possibly hit stop button
        if(Data(2)==9)||(Data(7)==9)
            disp("Torque value problem    Trq > 25Nm");
            set(handles.statusText,'String','Problem Trq Ctrl, Trq > 25 Nm');

        end

    else % it is a non data message
        command(msg, Data, handles);
    end
