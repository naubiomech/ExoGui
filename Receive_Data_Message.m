function [RLCount,LLCount] = Receive_Data_Message(message,RLCount,LLCount,hObject, eventdata, handles)

global GUI_Variables

if message(1) == 83 && message(length(message)-1) == 90
                indexes = find(message==44);
                if(indexes(1) == 2) % It means it is data message to plot and update signals
                    Data=zeros(1,length(indexes)-1);
                    for index_iterator = 1:(length(indexes)-1)
                        Data(index_iterator) = str2double(message((indexes(index_iterator)+1):(indexes(index_iterator+1)-1)));             
                    end
                    
                    GUI_Variables.RLTRQ(RLCount) = Data(1)/100;                 %Gets the new Torque Value and Stores it
                    GUI_Variables.RLFSR(RLCount) = Data(2); %state
                    GUI_Variables.RLSET(RLCount) = Data(3)/100; %save the torque set point
                    GUI_Variables.RLVOLT(RLCount) = Data(4)/100;
                    GUI_Variables.RLVOLT_H(RLCount) = Data(5)/100;
                    GUI_Variables.SIG1(RLCount) = Data(11)/100;
                    GUI_Variables.SIG3(RLCount) = Data(13)/100;
                    GUI_Variables.SIG4(RLCount) = Data(14)/100;
                    GUI_Variables.BASER(RLCount)=GUI_Variables.baser;
                                        
                    GUI_Variables.R_BAL_DYN_HEEL(RLCount)=GUI_Variables.R_Bal_dyn_Heel;
                    GUI_Variables.R_BAL_STEADY_HEEL(RLCount)=GUI_Variables.R_Bal_steady_Heel;
                    GUI_Variables.R_BAL_DYN_TOE(RLCount)=GUI_Variables.R_Bal_dyn_Toe;
                    GUI_Variables.R_BAL_STEADY_TOE(RLCount)=GUI_Variables.R_Bal_steady_Toe;
                    GUI_Variables.BASEL_BIOFB(RLCount)=GUI_Variables.basel_biofb;
                    
                    RLCount = RLCount + 1;                                         %Increments kneeCount                                          %Checks if Ankle Arduino Sent a new Torque Value 
                    GUI_Variables.RLCount = RLCount;
                    GUI_Variables.LLTRQ(LLCount) = Data(6)/100;            %Gets the new Torque Value and stores it
                    GUI_Variables.LLFSR(LLCount) = Data(7);
                    GUI_Variables.LLSET(LLCount) = Data(8)/100; %New to save also the set point
                    GUI_Variables.LLVOLT(LLCount) = Data(9)/100;
                    GUI_Variables.LLVOLT_H(LLCount) = Data(10)/100;
                    GUI_Variables.SIG2(LLCount) = Data(12)/100;
                    
                    GUI_Variables.BASEL(LLCount)=GUI_Variables.basel;
                                        
                    GUI_Variables.L_BAL_DYN_HEEL(LLCount)=GUI_Variables.L_Bal_dyn_Heel;
                    GUI_Variables.L_BAL_STEADY_HEEL(LLCount)=GUI_Variables.L_Bal_steady_Heel;
                    GUI_Variables.L_BAL_DYN_TOE(LLCount)=GUI_Variables.L_Bal_dyn_Toe;
                    GUI_Variables.L_BAL_STEADY_TOE(LLCount)=GUI_Variables.L_Bal_steady_Toe;

                    
                    
                    LLCount = LLCount + 1;  
                    GUI_Variables.LLCount = LLCount;
%                   pause(.000000001);                                               %Pauses to give time for the user to possibly hit stop button
                    if(Data(2)==9)||(Data(7)==9)
                       disp("Torque value problem    Trq > 25Nm");
                       set(handles.statusText,'String','Problem Trq Ctrl, Trq > 25 Nm');
                       
                    end
                    
                else % it is a non data message
                    command(message,indexes,hObject, eventdata, handles)
                end
                
end