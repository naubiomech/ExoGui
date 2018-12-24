function varargout = A_EXO_s(varargin)
% A_EXO_S MATLAB code for A_EXO_s.fig
%      A_EXO_S, by itself, creates a new A_EXO_S or raises the existing
%      singleton*.
%
%      H = A_EXO_S returns the handle to a new A_EXO_S or the handle to
%      the existing singleton*.
%
%      A_EXO_S('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in A_EXO_S.M with the given input arguments.
%
%      A_EXO_S('Property','Value',...) creates a new A_EXO_S or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before A_EXO_s_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to A_EXO_s_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help A_EXO_s

% Last Modified by GUIDE v2.5 13-Sep-2018 09:40:16

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @A_EXO_s_OpeningFcn, ...
                   'gui_OutputFcn',  @A_EXO_s_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before A_EXO_s is made visible.
function A_EXO_s_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to A_EXO_s (see VARARGIN)

handles.output = hObject;

% bt = Bluetooth('Exo_Bluetooth_5',1,'UserData',0,'InputBufferSize',2048*16); %Creates Bluetooth Object
% bt = Bluetooth('Exo_Bluetooth_2',1,'UserData',0,'InputBufferSize',2048*16); %Creates Bluetooth Object
% bt = Bluetooth('RNBT-0B45',1,'UserData',0,'InputBufferSize',2048*16*4); %Creates Bluetooth Object

BT_NAME={'Exo_Bluetooth_3','Capstone_Bluetooth_1','Exo_Bluetooth_2','Exo_High_Power'};
bt = Bluetooth(BT_NAME{1},1,'UserData',0,'InputBufferSize',2048*16*50); %Creates Bluetooth Object
disp('')%Exo_Bluetooth_2
        % Capstone_Bluetooth_1
str_uno=input('Would you use the arduino trigger? [y/n] ','s');

if (strcmp(str_uno,'y'))
    if exist('Uno')
        disp('Uno already exists in the workspace');
    else
        try
            disp('Opening Arduino connection');
            Uno=arduino;
            disp('Arduino opened');
        catch
            disp('Impossible to open arduino connection');
        end
    end
else
    Uno=0;
end

global GUI_Variables
GUI_Variables = struct('BT',bt,'Timer',NaN,'state',0,'RLTorque',NaN(1,60000), ...
                       'LLTorque',NaN(1,60000),'LLFSR',NaN(1,60000),'RLFSR',NaN(1,60000),...
                       'LLVOLT',NaN(1,60000),'RLVOLT',NaN(1,60000),'LLVOLT_H',NaN(1,60000),'RLVOLT_H',NaN(1,60000),'RLCount',1,'LLCount',1,...
                       'COUNT',0,'UNO',Uno,'flag_calib',0,'flag_start',0,'first_calib',0,...
                       'L_COUNT_SPEED',[],'R_COUNT_SPEED',[],...
                       'RLSET',NaN(1,60000),'LLSET',NaN(1,60000),'MEM',ones(1,3)*2,...
                       'SIG1',NaN(1,60000),'SIG2',NaN(1,60000),'SIG3',NaN(1,60000),'SIG4',NaN(1,60000),...
                       'BASEL',NaN(1,60000),'BASER',NaN(1,60000),'basel',0,'baser',0,'counter',0);
% Update handles structure
guidata(hObject, handles);


% --- Outputs from this function are returned to the command line.
function varargout = A_EXO_s_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in Start_Trial.
function Start_Trial_Callback(hObject, eventdata, handles)
%Make a further check of the connection before starting
disp("Before Starting we check the connections");
set(handles.statusText,'String','Checking Connections');
pause(0.01)

disp("Check BT")
Check_Bluetooth_Callback(hObject, eventdata, handles);
set(handles.statusText,'String','Connections Checked');
pause(0.01)

global GUI_Variables
bt = GUI_Variables.BT;

% if(bt.Status == "open")
%     try
% fwrite(bt,'C');
%     catch
%     end
% end
flushinput(bt);
flushoutput(bt);
allocated = 100000;
RLTorque = NaN(1,allocated);
LLTorque = NaN(1,allocated);
RLFSR = NaN(1,allocated);
LLFSR = NaN(1,allocated);

SIG1 = NaN(1,allocated);
SIG2 = NaN(1,allocated);
SIG3 = NaN(1,allocated);
SIG4 = NaN(1,allocated);
BASEL=NaN(1,allocated);
BASER=NaN(1,allocated);
basel=0;
baser=0;

RLCount = 1;                                                             %Initializes the starting space for the Vector holding the Torque Values
LLCount = 1;
GUI_Variables.LLFSR = LLFSR;
GUI_Variables.RLFSR = RLFSR;
GUI_Variables.RLTorque = RLTorque;
GUI_Variables.LLTorque = LLTorque;

LLSET = NaN(1,allocated);
RLSET = NaN(1,allocated);
GUI_Variables.RLSET = RLSET;
GUI_Variables.LLSET = LLSET;


LLVOLT = NaN(1,allocated);
RLVOLT = NaN(1,allocated);
GUI_Variables.LLVOLT = LLVOLT;
GUI_Variables.RLVOLT = RLVOLT;

LLVOLT_H = NaN(1,allocated);
RLVOLT_H = NaN(1,allocated);
GUI_Variables.LLVOLT_H = LLVOLT_H;
GUI_Variables.RLVOLT_H = RLVOLT_H;

GUI_Variables.SIG1=SIG1;
GUI_Variables.SIG2=SIG2;
GUI_Variables.SIG3=SIG3;
GUI_Variables.SIG4=SIG4;

GUI_Variables.BASEL=BASEL;
GUI_Variables.BASER=BASER;


GUI_Variables.counter=0;
set(handles.TRIG_NUM_TEXT,'String',0);

if(bt.Status == "open") 
    state = 1;
else
    state = 0;
end



if((bt.Status == "closed"))
    set(handles.statusText,'String','There are no Connections');
    pause(0.01);
    End_Trial_Callback(hObject, eventdata, handles)
    return
else
    GUI_Variables.state = state;
    disp("My state is ");
    disp(state);
end


%set(handles.L_Get_Setpoint,'Enable','off');
%set(handles.R_Get_Setpoint,'Enable','off');
%set(handles.Get_Smoothing,'Enable','off');
set(handles.Calibrate_FSR,'Enable','on');
set(handles.Calibrate_Torque,'Enable','off');
set(handles.Check_Memory,'Enable','off');
set(handles.Clean_Memory,'Enable','off');
%set(handles.L_Get_PID,'Enable','off');
%set(handles.R_Get_PID,'Enable','off');
%set(handles.L_Check_KF,'Enable','off');
%set(handles.R_Check_KF,'Enable','off');
%set(handles.L_Check_FSR_Th,'Enable','off');
%set(handles.R_Check_FSR_Th,'Enable','off');
set(handles.Start_Trial,'Enable','off');
set(handles.End_Trial,'Enable','on'); 

GUI_Variables.flag_start=1;

% set(handles.getRightPIDParamBut,'Enable','off');
%Disables Manual communication with the Arduino
% set(handles.calTorqBut,'Enable','off');                                   %Disables Manual communication with the Arduino
%Enables Manual "End Trial" with Bluetooth
% set(handles.waitRadioBut,'Enable','off');
% set(handles.noWaitRadioBut,'Enable','off');
% set(handles.getLeftAnkleValsBut,'Enable','off');
% set(handles.getRightAnkleValsBut,'Enable','off'); 
% 
% %FSR calib
% set(handles.calFSRBut,'Enable','on');
% %clean memory
% set(handles.Clean_Memory,'Enable','on');
% %check memory
% set(handles.Check_Sensor_Bias_Status,'Enable','off');
% %check memory
% set(handles.Get_Smoothing,'Enable','off');
% set(handles.Check_FSR_Th,'Enable','off');
% set(handles.L_Check_KF,'Enable','off');

% if get(handles.waitRadioBut,'Value') == 1
%     set(handles.statusText,'String','Waiting for external Switch');
%     pause(.001);
%     
%     Uno = arduino;
%     while readVoltage(Uno,'A0') < 3.8
%         pause(.00001);%Double check if This is necessary
%     end
%     set(handles.statusText,'String','External switch has been pushed');
% end

pause(.01);
if state == 1 
    fwrite(bt,char(69));                                                   %Sends ASCII character 69 to the Arduino, which the Arduino will
end
pause(.001);


start_count=0;

set(handles.statusText,'String','Trial has been started');
tic
if state == 1 % both connected
    disp('both connected')
    while strcmp(get(handles.Start_Trial,'Enable'), 'off')                         %Will Loop continuously until the "End Trial" button is pressed
        if RLCount > 1
            if toc > 0.9
                toc
                disp('')
                disp(' It takes too much time to communicate with the bluetooth')
                disp('')
                set(handles.End_Trial,'Enable','off');
                set(handles.statusText,'String','One of the Bluetooths has stopped streaming, all systems have shut down. Remaining Data is being saved.');
                pause(.0001);
                End_Trial_Callback(hObject, eventdata, handles)
            end
        end
        
        if GUI_Variables.flag_calib==1
            % % %         flushinput(bt);   % Not sure if this was commented? Giamma
            % please check
            if GUI_Variables.first_calib==0
                start_count=clock;
                start_count=start_count(6);
                disp("Start FSR Calib")
                GUI_Variables.first_calib=1;
            end
            %             disp(" Calib")
            v=clock;
            if (v(6)-start_count)>5
                set(handles.statusText,'String','Finished Calibrating the FSRs');
                pause(0.00000001);
                disp("End FSR Calib")
                GUI_Variables.flag_calib=0;
                GUI_Variables.first_calib=0;
            end
        end
        pause(.000000001); 
        if ((bt.bytesAvailable > 0))
            %           disp('saving in global variables');%Checks if Knee Arduino Sent a new Torque Value
            tic
            message = fgetl(bt);
            %             if message(1) == 83 && message(length(message)-1) == 90% && length(find(A==90)) == 1 && length(find(A==69)) == 1
            %                 indexes = find(message==44);
            %                 if(indexes(1) == 2) % It means it is data message to plot and update signals

            %                     for index_iterator = 1:(length(indexes)-1)
            %                         Data(index_iterator) = str2double(message((indexes(index_iterator)+1):(indexes(index_iterator+1)-1)));
            %                     end
            %                     GUI_Variables.RLTorque(RLCount) = Data(1);                 %Gets the new Torque Value and Stores it
            %                     GUI_Variables.RLFSR(RLCount) = Data(2);
            %                     GUI_Variables.RLSET(RLCount) = Data(3); %New to save also the set point
            %                     GUI_Variables.RLVOLT(RLCount) = Data(4);
            %                     GUI_Variables.SIG3(RLCount) = Data(9);
            %                     RLCount = RLCount + 1;                                         %Increments kneeCount                                          %Checks if Ankle Arduino Sent a new Torque Value
            %                     GUI_Variables.RLCount = RLCount;
            %                     GUI_Variables.LLTorque(LLCount) = Data(5);              %Gets the new Torque Value and stores it
            %                     GUI_Variables.LLFSR(LLCount) = Data(6);
            %                     GUI_Variables.LLSET(LLCount) = Data(7); %New to save also the set point
            %                     GUI_Variables.LLVOLT(LLCount) = Data(8);
            %                     GUI_Variables.SIG2(LLCount) = Data(10);
            %                     GUI_Variables.SIG1(LLCount) = Data(11);
            %                     LLCount = LLCount + 1;
            %                     GUI_Variables.LLCount = LLCount;
            % %                   pause(.000000001);                                               %Pauses to give time for the user to possibly hit stop button
            %                     if(Data(3)==9)||(Data(7)==9)
            %                        disp("Torque value problem    Trq > 25Nm");
            %                        set(handles.statusText,'String','Problem Trq Ctrl, Trq > 25 Nm');
            %
            %                     end
            %                 else % it is a non data message
            %                     command(message,indexes,hObject, eventdata, handles)
            %                 end
            %             end
            [RLCount,LLCount] = Receive_Data_Message(message,RLCount,LLCount,hObject, eventdata, handles);
            if(mod(RLCount,100) == 0)
                axes(handles.Bottom_Axes);
                whichPlotLeft = get(handles.Bottom_Graph,'Value');
                if whichPlotLeft == 1
                    plotThisLeft = GUI_Variables.RLTorque;
                    if RLCount <= 1000
                        plot(1:length(plotThisLeft), [plotThisLeft;GUI_Variables.RLSET]);
                        title("RL Torque");
                    end
                    if RLCount > 1005
                        plot((RLCount-1000):RLCount-1,[GUI_Variables.RLTorque((RLCount-1000):RLCount-1);GUI_Variables.RLSET((RLCount-1000):RLCount-1)]);
                        title("RL Torque");
                    end
                end
                if whichPlotLeft == 2
                    plotThisLeft = GUI_Variables.RLFSR;
                    if RLCount <= 1000
                        plot(1:length(plotThisLeft), plotThisLeft);
                        title("RL State");
                    end
                    if RLCount > 1005
                        plot((RLCount-1000):RLCount-1,GUI_Variables.RLFSR((RLCount-1000):RLCount-1));
                        title("RL State");
                    end
                end
                if whichPlotLeft == 3
                    plotThisLeft = GUI_Variables.LLTorque;
                    if LLCount <= 1000
                        plot(1:length(plotThisLeft), [plotThisLeft;GUI_Variables.LLSET]);
                        title("LL Torque");
                    end
                    if LLCount > 1005
                        plot((LLCount-1000):LLCount-1,[GUI_Variables.LLTorque((LLCount-1000):LLCount-1);GUI_Variables.LLSET((LLCount-1000):LLCount-1)]);
                        title("LL Torque");
                    end
                end
                if whichPlotLeft == 4
                    plotThisLeft = GUI_Variables.LLFSR;
                    if LLCount <= 1000
                        plot(1:length(plotThisLeft), plotThisLeft);
                        title("LL State");
                    end
                    if LLCount > 1005
                        plot((LLCount-1000):LLCount-1,GUI_Variables.LLFSR((LLCount-1000):LLCount-1));
                        title("LL State");
                    end
                end
                if whichPlotLeft == 5
                    %                     whichPlotLeft = GUI_Variables.LLVOLT;
                    
                    if LLCount <= 1000
                        plot(1:length(GUI_Variables.LLVOLT), [GUI_Variables.LLVOLT;GUI_Variables.LLVOLT_H;GUI_Variables.BASEL] );
                        title("LL Force Toe and Heel");
                    end
                    if LLCount > 1005
                        plot((LLCount-1000):LLCount-1,[GUI_Variables.LLVOLT((LLCount-1000):LLCount-1);GUI_Variables.LLVOLT_H((LLCount-1000):LLCount-1);GUI_Variables.BASEL((RLCount-1000):RLCount-1)]);
                        title("LL Force Toe and Heel");
                    end
                end
                if whichPlotLeft == 6
                    %                     whichPlotLeft = GUI_Variables.RLVOLT;
                    if RLCount <= 1000
                        plot(1:length(GUI_Variables.RLVOLT), [GUI_Variables.RLVOLT;GUI_Variables.RLVOLT_H; GUI_Variables.BASER]);
                        title("RL Force Toe and Heel");
                    end
                    if RLCount > 1005
                        plot((RLCount-1000):RLCount-1,[GUI_Variables.RLVOLT((RLCount-1000):RLCount-1); GUI_Variables.RLVOLT_H((RLCount-1000):RLCount-1); GUI_Variables.BASER((RLCount-1000):RLCount-1)]);
                        title("RL Force Toe and Heel");
                    end
                end   
                
                if whichPlotLeft == 7
                    if RLCount <= 1000
                        plot(1:length(GUI_Variables.SIG1), GUI_Variables.SIG1);
                        title("SIG1");
                    end
                    if RLCount > 1005
                        plot((RLCount-1000):RLCount-1,GUI_Variables.SIG1((RLCount-1000):RLCount-1));
                        title("SIG1");
                    end
                end
                
                if whichPlotLeft == 8
                    if RLCount <= 1000
                        plot(1:length(GUI_Variables.SIG2), GUI_Variables.SIG2);
                        title("SIG2");
                    end
                    if RLCount > 1005
                        plot((RLCount-1000):RLCount-1,GUI_Variables.SIG2((RLCount-1000):RLCount-1));
                        title("SIG2");
                    end
                end
                
                if whichPlotLeft == 9
                    if RLCount <= 1000
                        plot(1:length(GUI_Variables.SIG3), GUI_Variables.SIG3);
                        title("SIG3");
                    end
                    if RLCount > 1005
                        plot((RLCount-1000):RLCount-1,GUI_Variables.SIG3((RLCount-1000):RLCount-1));
                        title("SIG3");
                    end
                end
                
                if whichPlotLeft == 10
                    if RLCount <= 1000
                        plot(1:length(GUI_Variables.SIG4), GUI_Variables.SIG4);
                        title("SIG4");
                    end
                    if RLCount > 1005
                        plot((RLCount-1000):RLCount-1,GUI_Variables.SIG4((RLCount-1000):RLCount-1));
                        title("SIG4");
                    end
                end
                
                
                whichPlotRight = get(handles.Top_Graph,'Value');
                axes(handles.Top_Axes);
                if whichPlotRight == 1
                    plotThisRight = GUI_Variables.RLTorque;
                    if RLCount <= 1000
                        plot(1:length(plotThisRight), [plotThisRight;GUI_Variables.RLSET]);
                        title("RL Torque");
                    end
                    if RLCount > 1005
                        plot((RLCount-1000):RLCount-1,[GUI_Variables.RLTorque((RLCount-1000):RLCount-1);GUI_Variables.RLSET((RLCount-1000):RLCount-1)]);
                        title("RL Torque");
                    end
                end
                if whichPlotRight == 2
                    plotThisRight = GUI_Variables.RLFSR;
                    if RLCount <= 1000
                        plot(1:length(plotThisRight), plotThisRight);
                        title("RL State");
                    end
                    if RLCount > 1005
                        plot((RLCount-1000):RLCount-1,GUI_Variables.RLFSR((RLCount-1000):RLCount-1));
                        title("RL State");
                    end
                end
                if whichPlotRight == 3
                    plotThisRight = GUI_Variables.LLTorque;
                    if LLCount <= 1000
                        plot(1:length(plotThisRight), [plotThisRight;GUI_Variables.LLSET]);
                        title("LL Torque");
                    end
                    if LLCount > 1005
                        plot((LLCount-1000):LLCount-1,[GUI_Variables.LLTorque((LLCount-1000):LLCount-1);GUI_Variables.LLSET((RLCount-1000):RLCount-1)]);
                        title("LL Torque");
                    end
                end
                if whichPlotRight == 4
                    plotThisRight = GUI_Variables.LLFSR;
                    if LLCount <= 1000
                        plot(1:length(plotThisRight), plotThisRight);
                        title("LL State");
                    end
                    if LLCount > 1005
                        plot((LLCount-1000):LLCount-1,GUI_Variables.LLFSR((LLCount-1000):LLCount-1));
                        title("LL State");
                    end
                end
                if whichPlotRight == 5
                    %                     plotThisRight = GUI_Variables.LLVOLT;
                    if LLCount <= 1000
                        plot(1:length(GUI_Variables.LLVOLT), [GUI_Variables.LLVOLT;GUI_Variables.LLVOLT_H;GUI_Variables.BASEL]);
                        title("LL Force Toe and Heel");
                    end
                    if LLCount > 1005
                        plot((LLCount-1000):LLCount-1,[GUI_Variables.LLVOLT((LLCount-1000):LLCount-1);GUI_Variables.LLVOLT_H((LLCount-1000):LLCount-1);GUI_Variables.BASEL((LLCount-1000):LLCount-1)]);
                        title("LL Force Toe and Heel");
                    end
                end
                if whichPlotRight == 6
                    plotThisRight = GUI_Variables.RLVOLT;
                    if RLCount <= 1000
                        plot(1:length(plotThisRight), [GUI_Variables.RLVOLT;GUI_Variables.RLVOLT_H;GUI_Variables.BASER]);
                        title("RL Force Toe and Heel");
                    end
                    if RLCount > 1005
                        plot((RLCount-1000):RLCount-1,[GUI_Variables.RLVOLT((RLCount-1000):RLCount-1);GUI_Variables.RLVOLT_H((RLCount-1000):RLCount-1);GUI_Variables.BASER((LLCount-1000):LLCount-1)]);
                        title("RL Force Toe and Heel");
                    end
                end
                
                if whichPlotRight == 7
                    if RLCount <= 1000
                        plot(1:length(GUI_Variables.SIG1), GUI_Variables.SIG1);
                        title("SIG1");
                    end
                    if RLCount > 1005
                        plot((RLCount-1000):RLCount-1,GUI_Variables.SIG1((RLCount-1000):RLCount-1));
                        title("SIG1");
                    end
                end
                
                if whichPlotRight == 8
                    if RLCount <= 1000
                        plot(1:length(GUI_Variables.SIG2), GUI_Variables.SIG2);
                        title("SIG2");
                    end
                    if RLCount > 1005
                        plot((RLCount-1000):RLCount-1,GUI_Variables.SIG2((RLCount-1000):RLCount-1));
                        title("SIG2");
                    end
                end
                
                if whichPlotRight == 9
                    if RLCount <= 1000
                        plot(1:length(GUI_Variables.SIG3), GUI_Variables.SIG3);
                        title("SIG3");
                    end
                    if RLCount > 1005
                        plot((RLCount-1000):RLCount-1,GUI_Variables.SIG3((RLCount-1000):RLCount-1));
                        title("SIG3");
                    end
                end

                if whichPlotRight == 10
                    if RLCount <= 1000
                        plot(1:length(GUI_Variables.SIG4), GUI_Variables.SIG4);
                        title("SIG4");
                    end
                    if RLCount > 1005
                        plot((RLCount-1000):RLCount-1,GUI_Variables.SIG4((RLCount-1000):RLCount-1));
                        title("SIG4");
                    end
                end
                
                %toc
            end
        end
    end
end



% --- Executes on button press in End_Trial.
function End_Trial_Callback(hObject, eventdata, handles)
disp('')
disp('End Trial button pushed')
disp('')

global GUI_Variables
bt = GUI_Variables.BT;

if bt.Status=="open"
    LLTorque = GUI_Variables.LLTorque;
    LLFSR = GUI_Variables.LLFSR;
    RLTorque = GUI_Variables.RLTorque;
    RLFSR = GUI_Variables.RLFSR;
    RLSET = GUI_Variables.RLSET;
    LLSET = GUI_Variables.LLSET;
    RLCount = GUI_Variables.RLCount;
    LLCount = GUI_Variables.LLCount;
    RLVOLT = GUI_Variables.RLVOLT;
    LLVOLT = GUI_Variables.LLVOLT;
    RLVOLT_H = GUI_Variables.RLVOLT_H;
    LLVOLT_H = GUI_Variables.LLVOLT_H;
    SIG2 = GUI_Variables.SIG2;
    SIG1 = GUI_Variables.SIG1;
    state = GUI_Variables.state;
    SIG3 = GUI_Variables.SIG3;
    SIG4 = GUI_Variables.SIG4;
    BASEL=GUI_Variables.BASEL;
    BASER=GUI_Variables.BASER;
    % TRIG=GUI_Variables.TRIG;

    disp("");
    disp(" I am going to terminate the system, my current state is: ");
    disp(state);
    disp("");

    if state == 1
        try
            fwrite(bt,char(71));                                                   %Recognizes as stopping the motor and stop sending torque values
        catch
        end
    end

    try %if you turn off the motor before I push end trail this shouldn't be executed
        while((bt.bytesAvailable > 0))%I use the double while loop so I can incorporate a pause without having to pause every loop
                                      %I can pause after it think its finished looping (emptied the buffer), and then check if it should have finished looping (emptied the buffer) and if it is not finished, have it loop again
            while((bt.bytesAvailable > 0))     %While there are still torque values in the bluetooth buffer
                if(bt.bytesAvailable > 0)
                    message = fgetl(bt);
                    if message(1) == 83 && message(length(message)-1) == 90% && length(find(A==90)) == 1 && length(find(A==69)) == 1
                        indexes = find(message==44);
                        if(indexes(1) == 2)
                            for index_iterator = 1:(length(indexes)-1)
                                Data(index_iterator) = str2double(message((indexes(index_iterator)+1):(indexes(index_iterator+1)-1)));
                            end
                            %                     RLTorque(RLCount) = Data(1);                 %Gets the new Torque Value and Stores it
                            %                     RLFSR(RLCount) = Data(2);
                            %                     RLSET(RLCount) = Data(3); %New to save also the set point
                            %                     RLVOLT(RLCount) = Data(4);
                            %                     SIG3(RLCount) = Data(9);
                            %                     SIG1(RLCount) = Data(11);
                            %                     RLCount = RLCount + 1;                                         %Increments kneeCount                                          %Checks if Ankle Arduino Sent a new Torque Value
                            %                     RLCount = RLCount;
                            %                     LLTorque(LLCount) = Data(5);              %Gets the new Torque Value and stores it
                            %                     LLFSR(LLCount) = Data(6);
                            %                     LLSET(LLCount) = Data(7); %New to save also the set point
                            %                     LLVOLT(LLCount) = Data(8);
                            %                     SIG2(LLCount) = Data(10);
                            %
                            %                     LLCount = LLCount + 1;
                            %                     LLCount = LLCount;
                            RLTorque(RLCount) = Data(1);                 %Gets the new Torque Value and Stores it
                            RLFSR(RLCount) = Data(2);
                            RLSET(RLCount) = Data(3); %New to save also the set point
                            RLVOLT(RLCount) = Data(4);
                            RLVOLT_H(RLCount) = Data(5);
                            SIG1(RLCount) = Data(11);
                            SIG3(RLCount) = Data(13);
                            SIG4(RLCount) = Data(14);
                            BASER(RLCount)=GUI_Variables.baser;
                            RLCount = RLCount + 1;                                         %Increments kneeCount                                          %Checks if Ankle Arduino Sent a new Torque Value
                            RLCount = RLCount;
                            LLTorque(LLCount) = Data(6);              %Gets the new Torque Value and stores it
                            LLFSR(LLCount) = Data(7);
                            LLSET(LLCount) = Data(8); %New to save also the set point
                            LLVOLT(LLCount) = Data(9);
                            LLVOLT_H(LLCount) = Data(10);
                            SIG2(LLCount) = Data(12);
                            BASEL(LLCount)=GUI_Variables.basel;

                            %                   pause(.000000001);                                               %Pauses to give time for the user to possibly hit stop button
                        else
                            command(message,indexes,hObject, eventdata, handles)
                        end
                    end
                end                                                                  %Pause Long enough for any data in transit
            end
            pause(.5);
        end

    catch
    end


    LLTorque = LLTorque(1:(LLCount-1));
    RLTorque = RLTorque(1:(RLCount-1));
    RLFSR = RLFSR(1:(RLCount-1));
    LLFSR = LLFSR(1:(LLCount-1));

    LLSET = LLSET(1:(LLCount-1));
    RLSET = RLSET(1:(RLCount-1));

    LLVOLT = LLVOLT(1:(LLCount-1));
    RLVOLT = RLVOLT(1:(RLCount-1));

    LLVOLT_H = LLVOLT_H(1:(LLCount-1));
    RLVOLT_H = RLVOLT_H(1:(RLCount-1));

    SIG2 = SIG2(1:(LLCount-1));
    SIG1 = SIG1(1:(RLCount-1));
    SIG3 = SIG3(1:(RLCount-1));
    SIG4 = SIG4(1:(RLCount-1));
    BASEL=BASEL(1:(RLCount-1));
    BASER=BASER(1:(RLCount-1));

    if(state ==1)  %If both are on
        lengthDif = length(RLTorque) - length(LLTorque);                   %When both are on there is a possibility that one arduino sent more datapoints than the other
        disp(" ");
        disp(" Length DIF in state 1 : ");
        disp(lengthDif);                                                   %This would be caused by them not starting or stopping at the exact same time (to the nearest 10 ms)
        disp(" ");
        if lengthDif > 0                                                   %It looks for the difference in legnth of datapoints to determine which has more datapoints
            addOn = NaN(1,lengthDif);                                      %If the Left Or Right has more datapoints, it prepends the NaNs to start of the list with less datapoints
            LLTorque = [addOn, LLTorque];                                  %There is room for some error here, but the NaNs usually do not exceed 4, meaning that its at maximum,
            LLFSR = [addOn, LLFSR];                                        %40 ms out of sync
            LLSET = [addOn,LLSET];
            LLVOLT = [addOn,LLVOLT];
            LLVOLT_H = [addOn,LLVOLT_H];
            SIG2 =[addOn,SIG2];


        end
        if lengthDif < 0
            lengthDif = -lengthDif;
            addOn = NaN(1,lengthDif);
            RLTorque = [addOn,RLTorque];
            RLFSR = [addOn,RLFSR];
            RLSET = [addOn,RLSET];
            RLVOLT = [addOn,RLVOLT];
            RLVOLT_H = [addOn,RLVOLT_H];
            SIG1=[addOn,SIG1];
            SIG3=[addOn,SIG3];
            SIG4=[addOn,SIG4];
            BASEL=[BASEL;addOn];
            BASER=[BASER;addOn];
        end
        if lengthDif == 0                                                  %If they are the same length, everything is dandy
        end
    end

    if (RLCount>=LLCount)
        TRIG=zeros(size(RLTorque));
        L_SPEED=zeros(size(RLTorque));
        R_SPEED=zeros(size(RLTorque));
    else
        TRIG=zeros(size(LLTorque));
        L_SPEED=zeros(size(LLTorque));
        R_SPEED=zeros(size(LLTorque));
    end
    
    dt = .01;                                                              %Since the Arduino is set to send values every 10 ms, dt is .01 S
    t = 1:length(RLTorque);                                                %Creates a time Vector equal in length to the number of Torque Values Recieved
    t = t .* dt;                                                         %Scales the time Vector, knowing how often Arduino sends values,
    
    if isempty(GUI_Variables.COUNT) || (length(GUI_Variables.COUNT)==1) %beacuse the first is zero
    else
        %     disp('ok')
        count_trig_c=GUI_Variables.COUNT(2:end);

        for i=1:length(count_trig_c)
            %     count_trig_c(i)
            TRIG(count_trig_c(i))=1;
            % find(TRIG(count_trig_c(i))==1)
        end

    end

    if isempty(GUI_Variables.L_COUNT_SPEED)  %beacuse the first is zero
    else
        %     disp('ok')

        for i=1:size(GUI_Variables.L_COUNT_SPEED,1)
            %     count_trig_c(i)
            L_SPEED(GUI_Variables.L_COUNT_SPEED(i,1))=GUI_Variables.L_COUNT_SPEED(i,2);
            % find(TRIG(count_trig_c(i))==1)
        end
    end

    if isempty(GUI_Variables.R_COUNT_SPEED)  %beacuse the first is zero
    else
        %     disp('ok')
        for i=1:size(GUI_Variables.R_COUNT_SPEED,1)
            %     count_trig_c(i)
            R_SPEED(GUI_Variables.R_COUNT_SPEED(i,1))=GUI_Variables.R_COUNT_SPEED(i,2);
            % find(TRIG(count_trig_c(i))==1)
        end
    end

    % TRIG
    % size(TRIG)
    % size(LLSET)
    A = [t; RLTorque; RLFSR; RLSET; RLVOLT; RLVOLT_H; LLTorque; LLFSR; LLSET; LLVOLT; LLVOLT_H; TRIG; BASEL; BASER ;L_SPEED; R_SPEED; SIG1; SIG2; SIG3; SIG4];                                 %Creates a vector that holds the time and data
                                                                                                                                                                                               % A = [t; RLTorque; RLSET; LLTorque; LLSET]


    Filename = sprintf('%s_%d','Trial_Number_',bt.UserData);               %Creates a new filename called "Torque_#"
                                                                           %Where # is the trial number                                                                           
    fileID = fopen(Filename,'w');                                      %Actually creates that file
    pause(.01);
    fprintf(fileID,'\tTime\t\t RLTRQ\t\t RLFSR\t\t RLSET\t\t RLVOLT\t\t RLVOLT_H\t\t LLTRQ\t\t LLFSR\t\t LLSET\t\t LLVOLT\t\t LLVOLT_H\t\t TRIG\t\t BASEL\t\t BASER\t\t L_SPEED\t\t R_SPEED\t\t SIG1\t\t SIG2\t\t SIG3\t\t SIG4\t\t\n');
    fprintf(fileID,'%6.2f\t %12.2f\t %12.2f\t %12.2f\t %12.2f\t %12.2f\t %12.2f\t %12.2f\t %12.2f\t %12.2f\t %12.2f\t %12.2f\t %12.2f\t %12.2f\t %12.2f\t %12.2f\t %12.2f\t %12.2f\t %12.2f\t %12.2f\n',A);     %Puts the Time and Torque values in the txt columns
    fclose(fileID);

    if(bt.Status == "open")
        try
            fwrite(bt,'C');
        catch
        end
    end
    set(handles.Start_Trial,'Enable','on');
    flushinput(bt);
    pause(.5);
    flushoutput(bt);
    pause(.5);
    flushinput(bt);
    pause(.5);
    flushoutput(bt);
    pause(.5);

    % lkf=0;
    % rkf=0;
    %
    % lfsr=0;
    % rfsr=0;
    %
    % lkp=0;
    % lki=0;
    % lkd=0;
    %
    % rkp=0;
    % rki=0;
    % rkd=0;

    try
        [n1,n2,n3]=Get_Smoothing_Callback(hObject, eventdata, handles);
        pause(0.5);
        lfsr=L_Check_FSR_Th_Callback(hObject, eventdata, handles);
        % pause(0.5);
        rfsr=R_Check_FSR_Th_Callback(hObject, eventdata, handles);
        % pause(0.5);
        lkf=L_Check_KF_Callback(hObject, eventdata, handles);
        % pause(0.5);
        rkf=R_Check_KF_Callback(hObject, eventdata, handles);
        % pause(0.5);
        [lkp,lkd,lki]=L_Get_PID_Callback(hObject, eventdata, handles);
        % pause(0.5);
        [rkp,rkd,rki]=R_Get_PID_Callback(hObject, eventdata, handles);
        % pause(0.5);
    catch
    end

    Filename = sprintf('%s_%d','Parameters_Trial_Number_',bt.UserData);               %Creates a new filename called "Torque_#"
                                                                                      %Where # is the trial number
    fileID = fopen(Filename,'w');                                      %Actually creates that file
    pause(.01);
    % Smooth_params=['N1 = ',num2str(n1)];
    % Smooth_params_2=['N2 = ',num2str(n2),'N3 = ',num2str(n3),'\n'];
    % Filename = fprintf(fileID,Smooth_params);
    Filename = fprintf(fileID,['N1 = ',num2str(n1),'\n']);
    Filename = fprintf(fileID,['N2 = ',num2str(n2),'\n']);
    Filename = fprintf(fileID,['N3 = ',num2str(n3),'\n']);
    Filename = fprintf(fileID,['KF_LL = ',num2str(lkf),'\n']);
    Filename = fprintf(fileID,['KF_RL = ',num2str(rkf),'\n']);
    Filename = fprintf(fileID,['FSR_TH_LL = ',num2str(lfsr),'\n']);
    Filename = fprintf(fileID,['FSR_TH_RL = ',num2str(rfsr),'\n']);
    Filename = fprintf(fileID,['KP_L = ',num2str(lkp),'\n']);
    Filename = fprintf(fileID,['KD_L = ',num2str(lkd),'\n']);
    Filename = fprintf(fileID,['KI_L = ',num2str(lki),'\n']);
    Filename = fprintf(fileID,['KP_R = ',num2str(rkp),'\n']);
    Filename = fprintf(fileID,['KD_R = ',num2str(rkd),'\n']);
    Filename = fprintf(fileID,['KI_R = ',num2str(rki),'\n']);

    % ,'KD_L = ',num2str(lkd),'KI_L = ',num2str(lki),'\n']);
    % Filename = fprintf(fileID,['KP_R = ',num2str(rkp),'KD_R = ',num2str(rkd),'KI_R = ',num2str(rki),'\n']);

    fclose(fileID);
    %Closes the txt file
    set(handles.statusText,'String','Data has finished being Saved');
    RLTorque = [];
    RLFSR = [];
    LLTorque =[];
    LLFSR =[];

    LLSET =[];
    RLSET =[];
    TRIG=[];
    LLVOLT=[];
    RLVOLT=[];
    LLVOLT_H=[]
    ;
    RLVOLT_H=[];

    SIG1=[];
    SIG2=[];
    SIG3=[];
    SIG4=[];

    BASEL=[];
    BASER=[];

    LLCount = 1;
    RLCount = 1;
    bt.UserData = bt.UserData + 1;                                     %Increments the trial number


    set(handles.L_Get_Setpoint,'Enable','on');
    set(handles.R_Get_Setpoint,'Enable','on');
    set(handles.Get_Smoothing,'Enable','on');
    set(handles.Calibrate_FSR,'Enable','on');
    set(handles.Calibrate_Torque,'Enable','on');
    set(handles.Check_Memory,'Enable','on');
    set(handles.Clean_Memory,'Enable','on');
    set(handles.L_Get_PID,'Enable','on');
    set(handles.R_Get_PID,'Enable','on');
    set(handles.L_Check_KF,'Enable','on');
    set(handles.R_Check_KF,'Enable','on');
    set(handles.L_Check_FSR_Th,'Enable','on');
    set(handles.R_Check_FSR_Th,'Enable','on');
    set(handles.Start_Trial,'Enable','on');
    set(handles.End_Trial,'Enable','off');                                      %Disables the button to stop the trial
    set(handles.Start_Trial,'Enable','on');

    GUI_Variables.counter=0;
    set(handles.TRIG_NUM_TEXT,'String',0);

    GUI_Variables.RLTorque = RLTorque;
    GUI_Variables.LLTorque = LLTorque;
    GUI_Variables.RLFSR = RLFSR;
    GUI_Variables.LLFSR = LLFSR;
    GUI_Variables.RLCount = RLCount;
    GUI_Variables.LLCount = LLCount;

    GUI_Variables.LLSET = LLSET;
    GUI_Variables.RLSET = RLSET;
    GUI_Variables.LLVOLT = LLVOLT;
    GUI_Variables.RLVOLT = RLVOLT;
    GUI_Variables.LLVOLT_H = LLVOLT_H;
    GUI_Variables.RLVOLT_H = RLVOLT_H;
    % GUI_Variables.TRIG=TRIG;
    GUI_Variables.COUNT =0;

    GUI_Variables.flag_calib=0;
    GUI_Variables.flag_start=0;
    GUI_Variables.first_calib=0;

    GUI_Variables.L_COUNT_SPEED=[];
    GUI_Variables.R_COUNT_SPEED=[];

    GUI_Variables.SIG1=SIG1;
    GUI_Variables.SIG2=SIG2;
    GUI_Variables.SIG3=SIG3;
    GUI_Variables.SIG4=SIG4;

    GUI_Variables.BASEL=BASEL;
    GUI_Variables.BASER=BASER;

else
    set(handles.L_Get_Setpoint,'Enable','on');
    set(handles.R_Get_Setpoint,'Enable','on');
    set(handles.Get_Smoothing,'Enable','on');
    set(handles.Calibrate_FSR,'Enable','on');
    set(handles.Calibrate_Torque,'Enable','on');
    set(handles.Check_Memory,'Enable','on');
    set(handles.Clean_Memory,'Enable','on');
    set(handles.L_Get_PID,'Enable','on');
    set(handles.R_Get_PID,'Enable','on');
    set(handles.L_Check_KF,'Enable','on');
    set(handles.R_Check_KF,'Enable','on');
    set(handles.L_Check_FSR_Th,'Enable','on');
    set(handles.R_Check_FSR_Th,'Enable','on');
    set(handles.Start_Trial,'Enable','on');
    set(handles.End_Trial,'Enable','off');                                      %Disables the button to stop the trial
    set(handles.Start_Trial,'Enable','on');
    disp("System not connected");
    set(handles.statusText,'String','System not connected');
end



% --- Executes on button press in Calibrate_Torque.
function Calibrate_Torque_Callback(hObject, eventdata, handles)
% hObject    handle to Calibrate_Torque (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.statusText,'String',"Calibrating the Torque Sensors! <(^_^<)");
pause(.01);
global GUI_Variables
bt = GUI_Variables.BT;
if(bt.Status == "open")
    try
        fwrite(bt,char(72));
    catch
    end
end
% pause(2);
% set(handles.statusText,'String',"The Torque Sensors have been Calibrated! (>^_^)>");
pause(2);
set(handles.statusText,'String',"The Torque Sensors have been Calibrated!");

% --- Executes on button press in Calibrate_FSR.
function Calibrate_FSR_Callback(hObject, eventdata, handles)
% hObject    handle to Calibrate_FSR (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global GUI_Variables

bt = GUI_Variables.BT;

if(bt.status == "open")
    try
        fwrite(bt,char(76))
    catch
    end
end
set(handles.statusText,'String','Calibrating the FSRs');
if GUI_Variables.flag_start==0
    pause(5);
    set(handles.statusText,'String','Finished Calibrating the FSRs');
else
    GUI_Variables.flag_calib=1;

end


% --- Executes on button press in Check_Memory.
function Check_Memory_Callback(hObject, eventdata, handles)
disp("Checking Memory Status")
set(handles.statusText,'String','Checking Memory Status');
pause(0.01);

global GUI_Variables
bt = GUI_Variables.BT;
mem=GUI_Variables.MEM;
if(bt.Status == "open")
    fwrite(bt,char(60));
    try
        message = fgetl(bt);
        indexes = find(message==44);
    catch MER1
        MER1
    end
    if message(1) == 83 && message(length(message)-1) == 90 && message(2) == '<'
        check_torque = str2double(message((indexes(1)+1):(indexes(2)-1)));
        check_FSR = str2double(message((indexes(2)+1):(indexes(3)-1)));
        check_EXP = str2double(message((indexes(3)+1):(indexes(4)-1)));
        
        if(check_torque == 1)
            set(handles.axes10,'Color',[0 1 0])
            mem(2)=1;
        else
            set(handles.axes10,'Color',[1 0 0])
            mem(2)=0;
        end
        if(check_FSR == 1)
            set(handles.axes8,'Color',[0 1 0])
            mem(1)=1;
        else
            set(handles.axes8,'Color',[1 0 0])
            mem(1)=0;
        end
        if(check_EXP == 1)
            set(handles.EXP_Params_axes,'Color',[0 1 0])
            mem(3)=1;
        else
            set(handles.EXP_Params_axes,'Color',[1 0 0])
            mem(3)=0;
        end
    end
else
    disp("the status bt is not opened")
    set(handles.axes8,'Color',[0 0 0])
    set(handles.axes10,'Color',[0 0 0])
    set(handles.EXP_Params_axes,'Color',[0 0 0])
    mem=ones(1,3)*2;
end
GUI_Variables.MEM=mem;
set(handles.statusText,'String','Memory Status Checked');

% --- Executes on button press in Clean_Memory.
function Clean_Memory_Callback(hObject, eventdata, handles)
% hObject    handle to Clean_Memory (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global GUI_Variables
bt = GUI_Variables.BT;

set(handles.statusText,'String','Clearing Memory');
pause(0.1);

if(bt.Status == "open")

    fwrite(bt,char(62));
    disp(" Clean_Memory_Callback")
    pause(0.1);

end
set(handles.statusText,'String','Memory Cleared');


% --- Executes on button press in L_Check_KF.
function lkf=L_Check_KF_Callback(hObject, eventdata, handles)
global GUI_Variables;
bt = GUI_Variables.BT; 
lkf=0;
if (bt.Status=="open")
    %    try
    fwrite(bt,'`'); %send the character "`"
    if (strcmp(get(handles.Start_Trial,'Enable'), 'on'))
        message = fgetl(bt)
        if message(1) == 83 && message(length(message)-1) == 90 && message(2) == '`'
            indexes = find(message==44);
            KF_LL = str2double(message((indexes(1)+1):(indexes(2)-1)));
            
            %Curr_KF = str2double(fgets(bt));                                              %Gets the Current Arduino Torque Setpoint
            set(handles.L_Check_KF_Text,'String',KF_LL);
            disp("Left Current KF ");
            disp(KF_LL);
            lkf=KF_LL;
        else
            disp("Something wrong in KF");
            set(handles.L_Check_KF_Text,'String',"NaN");
        end
    end
    %    catch
    %        disp("Impossible to know KF");
    %        set(handles.L_Check_KF_Text,'String',"NaN");
    %    end
end

if (bt.Status=="closed") 
    disp("Impossible to know KF");
    set(handles.L_Check_KF_Text,'String',"NaN");
end

% --- Executes on button press in L_Send_KF.
function L_Send_KF_Callback(hObject, eventdata, handles)
% hObject    handle to L_Send_KF (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

new_KF = str2double(get(handles.L_Send_KF_Edit,'String'));         %Gets the Value entered into the edit Box in the G

global GUI_Variables
state=GUI_Variables.state;
% disp(state);

bt = GUI_Variables.BT; 

if (bt.Status=="open")
    try
        fwrite(bt,char(95)); %send the character "_"
        fwrite(bt,new_KF,'double');
        % str=["Send new Right KF ", num2str(new_KF)];
        disp("Send new Left KF ");
        disp(new_KF);

    catch
        disp("Impossible to write on bt the new KF");
    end
end


function L_Send_KF_Edit_Callback(hObject, eventdata, handles)
% hObject    handle to L_Send_KF_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of L_Send_KF_Edit as text
%        str2double(get(hObject,'String')) returns contents of L_Send_KF_Edit as a double


% --- Executes during object creation, after setting all properties.
function L_Send_KF_Edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to L_Send_KF_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in Top_Graph.
function Top_Graph_Callback(hObject, eventdata, handles)
% hObject    handle to Top_Graph (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns Top_Graph contents as cell array
%        contents{get(hObject,'Value')} returns selected item from Top_Graph


% --- Executes during object creation, after setting all properties.
function Top_Graph_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Top_Graph (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in Bottom_Graph.
function Bottom_Graph_Callback(hObject, eventdata, handles)
% hObject    handle to Bottom_Graph (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns Bottom_Graph contents as cell array
%        contents{get(hObject,'Value')} returns selected item from Bottom_Graph


% --- Executes during object creation, after setting all properties.
function Bottom_Graph_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Bottom_Graph (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in Send_Trig.
function Send_Trig_Callback(hObject, eventdata, handles)
% hObject    handle to Send_Trig (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% tic
global GUI_Variables
RLCount=GUI_Variables.RLCount;
LLCount=GUI_Variables.LLCount;
state=GUI_Variables.state;
count_trig=1;

if state==1
    if RLCount>=LLCount
        count_trig=RLCount;
    else
        count_trig=LLCount;
    end
end

% GUI_Variables.TRIG(count_trig)=writeDigitalPin(GUI_Variables.UNO, 'D0', 5);
try
    tic
    writeDigitalPin(GUI_Variables.UNO, 'D5', 1);
    toc
catch
end
GUI_Variables.COUNT=[GUI_Variables.COUNT;count_trig];
GUI_Variables.counter=GUI_Variables.counter+1;
cane=GUI_Variables.counter;
% disp(cane)
set(handles.TRIG_NUM_TEXT,'String',num2str(cane));
try
    writeDigitalPin(GUI_Variables.UNO, 'D5', 0);
catch
end
% toc

% --- Executes on button press in Check_Bluetooth.
function valBT=Check_Bluetooth_Callback(hObject, eventdata, handles)
global GUI_Variables
bt = GUI_Variables.BT;
pause(.01);
set(handles.statusText,'String',"Checking Bluetooth Connection");
pause(.01);
mem=GUI_Variables.MEM;
try
    fwrite(bt,char(78))
    try
        message = fgetl(bt);
        if message(1) == 83 && message(length(message)-1) == 90 && message(2) == 'N'              
            indexes = find(message==44);
            check1 = str2double(message((indexes(1)+1):(indexes(2)-1)));
            check2 = str2double(message((indexes(2)+1):(indexes(3)-1)));
            check3 = str2double(message((indexes(3)+1):(indexes(4)-1)));
            if(check1 == 0 && check2 == 1 && check3 == 2)
                set(handles.statusText,'String',"Working as Expected!");
                set(handles.flag_bluetooth,'Color',[0 1 0]);

                if mem(1)==0
                    set(handles.axes8,'Color',[1 0 0])
                elseif mem(1)==1
                    set(handles.axes8,'Color',[0 1 0])
                else
                    set(handles.axes8,'Color',[0 0 1])
                end
                if mem(2)==0
                    set(handles.axes10,'Color',[1 0 0])
                elseif mem(2)==1
                    set(handles.axes10,'Color',[0 1 0])
                else
                    set(handles.axes10,'Color',[0 0 1])
                end
                if mem(3)==0
                    set(handles.EXP_Params_axes,'Color',[1 0 0])
                elseif mem(3)==1
                    set(handles.EXP_Params_axes,'Color',[0 1 0])
                else
                    set(handles.EXP_Params_axes,'Color',[0 0 1])
                end
                %         set(handles.axes10,'Color',[0 0 1])
                %         set(handles.EXP_Params_axes,'Color',[0 0 1])
                valBT=1;
                
            else
                try
                    set(handles.statusText,'String',"A problem Occured and Bt has been closed!");
                    set(handles.flag_bluetooth,'Color',[1 0 0]);
                    set(handles.axes8,'Color',[0 0 0])
                    set(handles.axes10,'Color',[0 0 0])
                    set(handles.EXP_Params_axes,'Color',[0 0 0])
                    valBT=0;
                    fclose(bt);                   
                catch
                end
            end
        end
    catch ME_right_read
        set(handles.statusText,'String',"A problem Occured and Bt has been closed!");
        set(handles.flag_bluetooth,'Color',[1 0 0]);
        set(handles.axes8,'Color',[0 0 0])
        set(handles.axes10,'Color',[0 0 0])
        set(handles.EXP_Params_axes,'Color',[0 0 0])
        valBT=0;
        fclose(bt);
    end
catch
    try
        set(handles.flag_bluetooth,'Color',[1 0 0]);
        set(handles.axes8,'Color',[0 0 0])
        set(handles.axes10,'Color',[0 0 0])
        set(handles.EXP_Params_axes,'Color',[0 0 0])
        valBT=0;
        fclose(bt);
        set(handles.statusText,'String',"MATLAB was unable to communicate with the Bluetooth");
    catch
    end
end

% --- Executes on button press in Connect_BT.
function Connect_BT_Callback(hObject, eventdata, handles)
% hObject    handle to Connect_BT (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global GUI_Variables
bt = GUI_Variables.BT;
fprintf('Attempting to make a connection to the bluetooth\n');
set(handles.statusText,'String',"Attempting to make a Connection to the Bluetooth!");
pause(.01);


try
    fopen(bt);                                                           %Attempts to Make a connection to Bluetooth Object
catch ME_right_open
    set(handles.flag_bluetooth,'Color',[1 0 0]);
    set(handles.axes8,'Color',[0 0 0])
    set(handles.axes10,'Color',[0 0 0])
    set(handles.EXP_Params_axes,'Color',[0 0 0])
    %     ME_right_open
    %If fopen Fails do Nothing
end                                                            %Makes a connection to Bluetooth Object

if(bt.status == "open")
    set(handles.flag_bluetooth,'Color',[0 1 0]);
    set(handles.axes8,'Color',[0 0 1])
    set(handles.axes10,'Color',[0 0 1])
    set(handles.EXP_Params_axes,'Color',[0 0 1])
    fprintf("Made a connection to the Right Ankle bluetooth!\n");
    set(handles.statusText,'String',"Made a Connection to the Right Ankle Bluetooth!");
end

if(bt.status == "closed")
    set(handles.flag_bluetooth,'Color',[1 0 0]);
    set(handles.axes8,'Color',[0 0 0])
    set(handles.axes10,'Color',[0 0 0])
    set(handles.EXP_Params_axes,'Color',[0 0 0])
    set(handles.statusText,'String',"Could Not Connect to the Right Ankle Bluetooth :(  Try Again! (If it fails 3+ times attempt a power cycle)");
end


% --- Executes on selection change in L_List.
function L_List_Callback(hObject, eventdata, handles)
% hObject    handle to L_List (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns L_List contents as cell array
%        contents{get(hObject,'Value')} returns selected item from L_List
selectMode = get(handles.L_List,'Value');
if selectMode == 1
    set(handles.L_Torque,'Visible','on');
    set(handles.L_PID,'Visible','off');
    set(handles.L_Adj,'Visible','off');
    set(handles.L_Proportional_Ctrl,'Visible','off');
end
if selectMode == 2
    set(handles.L_Torque,'Visible','off');
    set(handles.L_PID,'Visible','on');
    set(handles.L_Adj,'Visible','off');
    set(handles.L_Proportional_Ctrl,'Visible','off');
end
if selectMode == 3
    set(handles.L_Torque,'Visible','off');
    set(handles.L_PID,'Visible','off');
    set(handles.L_Adj,'Visible','on');
    set(handles.L_Proportional_Ctrl,'Visible','off');
end
if selectMode == 4
    set(handles.L_Torque,'Visible','off');
    set(handles.L_PID,'Visible','off');
    set(handles.L_Adj,'Visible','off');
    set(handles.L_Proportional_Ctrl,'Visible','on');
end


% --- Executes during object creation, after setting all properties.
function L_List_CreateFcn(hObject, eventdata, handles)
% hObject    handle to L_List (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in R_list.
function R_list_Callback(hObject, eventdata, handles)
% hObject    handle to R_list (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns R_list contents as cell array
%        contents{get(hObject,'Value')} returns selected item from R_list
selectMode = get(handles.R_list,'Value');
if selectMode == 1
    set(handles.R_Torque,'Visible','on');
    set(handles.R_PID,'Visible','off');
    set(handles.R_Adj,'Visible','off');
    set(handles.R_Smoothing,'Visible','off');
    set(handles.R_Proportional_Ctrl,'Visible','off');
end
if selectMode == 2
    set(handles.R_Torque,'Visible','off');
    set(handles.R_PID,'Visible','on');
    set(handles.R_Adj,'Visible','off');
    set(handles.R_Smoothing,'Visible','off');
    set(handles.R_Proportional_Ctrl,'Visible','off');
end
if selectMode == 3
    set(handles.R_Torque,'Visible','off');
    set(handles.R_PID,'Visible','off');
    set(handles.R_Adj,'Visible','on');
    set(handles.R_Smoothing,'Visible','off');
    set(handles.R_Proportional_Ctrl,'Visible','off');
end
if selectMode == 4
    set(handles.R_Torque,'Visible','off');
    set(handles.R_PID,'Visible','off');
    set(handles.R_Adj,'Visible','off');
    set(handles.R_Smoothing,'Visible','on');
    set(handles.R_Proportional_Ctrl,'Visible','off');
end
if selectMode == 5
    set(handles.R_Torque,'Visible','off');
    set(handles.R_PID,'Visible','off');
    set(handles.R_Adj,'Visible','off');
    set(handles.R_Smoothing,'Visible','off');
    set(handles.R_Proportional_Ctrl,'Visible','on');
end

% --- Executes during object creation, after setting all properties.
function R_list_CreateFcn(hObject, eventdata, handles)
% hObject    handle to R_list (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function R_Ki_Edit_Callback(hObject, eventdata, handles)
% hObject    handle to R_Ki_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of R_Ki_Edit as text
%        str2double(get(hObject,'String')) returns contents of R_Ki_Edit as a double


% --- Executes during object creation, after setting all properties.
function R_Ki_Edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to R_Ki_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function R_Kd_Edit_Callback(hObject, eventdata, handles)
% hObject    handle to R_Kd_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of R_Kd_Edit as text
%        str2double(get(hObject,'String')) returns contents of R_Kd_Edit as a double


% --- Executes during object creation, after setting all properties.
function R_Kd_Edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to R_Kd_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function R_Kp_Edit_Callback(hObject, eventdata, handles)
% hObject    handle to R_Kp_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of R_Kp_Edit as text
%        str2double(get(hObject,'String')) returns contents of R_Kp_Edit as a double


% --- Executes during object creation, after setting all properties.
function R_Kp_Edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to R_Kp_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in R_Set_PID.
function R_Set_PID_Callback(hObject, eventdata, handles)
% hObject    handle to R_Set_PID (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% SEND 'm'
global GUI_Variables
bt = GUI_Variables.BT;
if(bt.Status=="open")
    fwrite(bt,char(109));
    kp = str2double(get(handles.R_Kp_Edit,'String'));               %Gets the Value entered into the edit Box in the G
    kd = str2double(get(handles.R_Kd_Edit,'String'));
    ki = str2double(get(handles.R_Ki_Edit,'String'));


    if size(ki,1)>1
        k1=str2double(get(handles.R_Ki_Edit,'String'));
        if isnan(k1(1))
            ki=k1(2);
        else
            ki=k1(1);
        end
    end

    if size(kp,1)>1
        k1=str2double(get(handles.R_Kp_Edit,'String'));
        if isnan(k1(1))
            kp=k1(2);
        else
            kp=k1(1);
        end
    end

    if size(kd,1)>1
        k1=str2double(get(handles.R_Kd_Edit,'String'));
        if isnan(k1(1))
            kd=k1(2);
        else
            kd=k1(1);
        end
    end
    
    disp(' New R PID gain')
    disp(kp)
    disp(kd)
    disp(ki)

    fwrite(bt,kp,'double');                                   %Sends the new Torque Value to Arduino
    fwrite(bt,kd,'double');
    fwrite(bt,ki,'double');
end

% --- Executes on button press in R_Get_PID.
function [rkp,rkd,rki]=R_Get_PID_Callback(hObject, eventdata, handles)
% SEND 'k' char 107
global GUI_Variables
bt = GUI_Variables.BT;

rkp=0;
rkd=0;
rki=0;

if(bt.Status=="open")
    fwrite(bt,char(107));
    if(strcmp(get(handles.Start_Trial,'Enable'), 'on') )
        message = fgetl(bt);
        if message(1) == 83 && message(length(message)-1) == 90 && message(2) == 'k'
            indexes = find(message==44);
            rkp = str2double(message((indexes(1)+1):(indexes(2)-1)));
            rkd = str2double(message((indexes(2)+1):(indexes(3)-1)));
            rki = str2double(message((indexes(3)+1):(indexes(4)-1))); 
            set(handles.R_Kp_text,'String',rkp);
            set(handles.R_Kd_Text,'String',rkd);
            set(handles.R_Ki_Text,'String',rki);
        end
    end
end

% --- Executes on button press in L_Get_PID.
function [lkp,lkd,lki]=L_Get_PID_Callback(hObject, eventdata, handles)
% SEND 'K' char 75
global GUI_Variables
bt = GUI_Variables.BT;

lkp=0;
lkd=0;
lki=0;

if(bt.Status=="open")
    fwrite(bt,char(75));
    if(strcmp(get(handles.Start_Trial,'Enable'), 'on'))
        message = fgetl(bt);
        if message(1) == 83 && message(length(message)-1) == 90 && message(2) == 'K'
            indexes = find(message==44);
            lkp = str2double(message((indexes(1)+1):(indexes(2)-1)));
            lkd = str2double(message((indexes(2)+1):(indexes(3)-1)));
            lki = str2double(message((indexes(3)+1):(indexes(4)-1)));
        end
        %lkp = str2double(fgets(bt));
        %lkd = str2double(fgets(bt));
        %lki = str2double(fgets(bt));
        set(handles.L_Kp_text,'String',lkp);
        set(handles.L_Kd_text,'String',lkd);
        set(handles.L_Ki_text,'String',lki);
    end
end
% --- Executes on button press in L_Set_PID.
function L_Set_PID_Callback(hObject, eventdata, handles)
% hObject    handle to L_Set_PID (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% SEND 'M'
global GUI_Variables
bt = GUI_Variables.BT;
if(bt.Status=="open")
    % fwrite(bt,char(77));
    fwrite(bt,'M');
    kp = str2double(get(handles.L_Kp_Edit,'String'));               %Gets the Value entered into the edit Box in the G
    kd = str2double(get(handles.L_Kd_Edit,'String'));
    ki = str2double(get(handles.L_Ki_Edit,'String'));

    if size(ki,1)>1
        k1=str2double(get(handles.L_Ki_Edit,'String'));
        if isnan(k1(1))
            ki=k1(2);
        else
            ki=k1(1);
        end
    end

    if size(kp,1)>1
        k1=str2double(get(handles.L_Kp_Edit,'String'));
        if isnan(k1(1))
            kp=k1(2);
        else
            kp=k1(1);
        end
    end

    if size(kd,1)>1
        k1=str2double(get(handles.L_Kd_Edit,'String'));
        if isnan(k1(1))
            kd=k1(2);
        else
            kd=k1(1);
        end
    end
    
    disp(' New L PID gain')
    disp(kp)
    disp(kd)
    disp(ki)


    fwrite(bt,kp,'double');                                   %Sends the new Torque Value to Arduino
    fwrite(bt,kd,'double');
    fwrite(bt,ki,'double');
end

function L_Kp_Edit_Callback(hObject, eventdata, handles)
% hObject    handle to L_Kp_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of L_Kp_Edit as text
%        str2double(get(hObject,'String')) returns contents of L_Kp_Edit as a double


% --- Executes during object creation, after setting all properties.
function L_Kp_Edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to L_Kp_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function L_Kd_Edit_Callback(hObject, eventdata, handles)
% hObject    handle to L_Kd_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of L_Kd_Edit as text
%        str2double(get(hObject,'String')) returns contents of L_Kd_Edit as a double


% --- Executes during object creation, after setting all properties.
function L_Kd_Edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to L_Kd_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in R_Set_Setpoint.
function R_Set_Setpoint_Callback(hObject, eventdata, handles)
% hObject    handle to R_Set_Setpoint (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% SEND f
global GUI_Variables
bt = GUI_Variables.BT;

if(bt.Status=="open")
    fwrite(bt,char(102));
end

%set(handles.plotBut,'Enable','off');
NewSetpoint = str2double(get(handles.R_Setpoint_Edit,'String'));               %Gets the Value entered into the edit Box in the G
fwrite(bt,NewSetpoint,'double'); 
NewSetpoint_Dorsi = str2double(get(handles.R_Setpoint_Dorsi_Edit,'String'));               %Gets the Value entered into the edit Box in the G
fwrite(bt,NewSetpoint_Dorsi,'double'); 

function R_Setpoint_Edit_Callback(hObject, eventdata, handles)
% hObject    handle to R_Setpoint_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of R_Setpoint_Edit as text
%        str2double(get(hObject,'String')) returns contents of R_Setpoint_Edit as a double


% --- Executes during object creation, after setting all properties.
function R_Setpoint_Edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to R_Setpoint_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in R_Get_Setpoint.
function R_Get_Setpoint_Callback(hObject, eventdata, handles)
%  SEND 'd'
global GUI_Variables
bt = GUI_Variables.BT;

if(bt.Status=="open")
    fwrite(bt,char(100));
end

if(strcmp(get(handles.Start_Trial,'Enable'), 'on'))
    message = fgetl(bt);
    if message(1) == 83 && message(length(message)-1) == 90 && message(2) == 'd'
        indexes = find(message==44);
        Setpoint_RL = str2double(message((indexes(1)+1):(indexes(2)-1)));
        Setpoint_Dorsi_RL = str2double(message((indexes(2)+1):(indexes(3)-1)));
        disp("Right New Setpoint")
        disp(Setpoint_RL)
        disp(Setpoint_Dorsi_RL)
        set(handles.R_Setpoint_Text,'String',Setpoint_RL);
        set(handles.R_Setpoint_Dorsi_Text,'String',Setpoint_Dorsi_RL);
    end

end
% --- Executes on button press in L_Get_Setpoint.
function L_Get_Setpoint_Callback(hObject, eventdata, handles)
% SEND 'D'
global GUI_Variables
bt = GUI_Variables.BT;

fwrite(bt,'D');                                                  %Sends the character corresponding to ASCII value 68 to Arduino
                                                                 %Which the Arduino understands to send parameters back
                                                                 %Setpoint_LL = fgets(bt);                                              %Gets the Current Arduino Torque Setpoint
if(strcmp(get(handles.Start_Trial,'Enable'), 'on'))
    message = fgetl(bt);
    if message(1) == 83 && message(length(message)-1) == 90 && message(2) == 'D'
        indexes = find(message==44);
        Setpoint_LL = str2double(message((indexes(1)+1):(indexes(2)-1)));
        Setpoint_Dorsi_LL = str2double(message((indexes(2)+1):(indexes(3)-1)));
        set(handles.L_Setpoint_Text,'String',Setpoint_LL);
        set(handles.L_Setpoint_Dorsi_Text,'String',Setpoint_Dorsi_LL);
        disp("Left New Setpoint")
        disp(Setpoint_LL)
    end
    
end


function L_Setpoint_Edit_Callback(hObject, eventdata, handles)
% hObject    handle to L_Setpoint_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of L_Setpoint_Edit as text
%        str2double(get(hObject,'String')) returns contents of L_Setpoint_Edit as a double



% --- Executes during object creation, after setting all properties.
function L_Setpoint_Edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to L_Setpoint_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in L_Set_Setpoint.
function L_Set_Setpoint_Callback(hObject, eventdata, handles)
% hObject    handle to L_Set_Setpoint (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% SEND 'F'
global GUI_Variables
bt = GUI_Variables.BT;

if(bt.Status=="open")
    fwrite(bt,char(70));
end

%set(handles.plotBut,'Enable','off');
NewSetpoint = str2double(get(handles.L_Setpoint_Edit,'String'));               %Gets the Value entered into the edit Box in the G
fwrite(bt,NewSetpoint,'double'); 
NewSetpoint_Dorsi = str2double(get(handles.L_Setpoint_Dorsi_Edit,'String'));
fwrite(bt,NewSetpoint_Dorsi,'double'); 

GUI_Variables.Setpoint=NewSetpoint;
axes(handles.PROP_FUNCTION_axes);
hold off
x=0.4:0.01:1.2;
plot(x,GUI_Variables.Setpoint*(128.1*x.^2-50.82*x+22.06)/(128.1-50.82+22.06));
hold on
plot([1 1],ylim,'-.k')

plot(xlim,[NewSetpoint NewSetpoint],'-.k')

% --- Executes on button press in Get_Smoothing.
function [n1,n2,n3]=Get_Smoothing_Callback(hObject, eventdata, handles)
% SEND '(' to get the smoothing
global GUI_Variables
bt = GUI_Variables.BT;
n1=0;
n2=0;
n3=0;
if (bt.Status=="open")
    try
        fwrite(bt,'(');
        if(strcmp(get(handles.Start_Trial,'Enable'), 'on') )
            message = fgetl(bt)
            if message(1) == 83 && message(length(message)-1) == 90 && message(2) == '('
                indexes = find(message==44);
                N1 = str2double(message((indexes(1)+1):(indexes(2)-1)));
                N2 = str2double(message((indexes(2)+1):(indexes(3)-1)));
                N3 = str2double(message((indexes(3)+1):(indexes(4)-1)));
                n1=N1;
                n2=N2;
                n3=N3;
                set(handles.N1_Text,'String',N1);
                set(handles.N2_Text,'String',N2);
                set(handles.N3_Text,'String',N3);
            end

        end
    catch
        %     else
        disp("Impossible to get shaping parameters from bt");
        set(handles.N1_Text,'String',"NaN");
        set(handles.N2_Text,'String',"NaN");
        set(handles.N3_Text,'String',"NaN");
    end
end

% --- Executes on button press in Set_Smoothing.
function Set_Smoothing_Callback(hObject, eventdata, handles)
% hObject    handle to Set_Smoothing (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global GUI_Variables
bt = GUI_Variables.BT;

% SEND '(' to get the smoothing

if (bt.Status=="open")
    try
        fwrite(bt,')');
        N1 = str2double(get(handles.N1_Edit,'String'));               %Gets the Value entered into the edit Box in the G
        N2 = str2double(get(handles.N2_Edit,'String'));
        N3 = str2double(get(handles.N3_Edit,'String'));
        disp('Smoothing');
        disp(N1)
        disp(N2)
        disp(N3)
        fwrite(bt,N1,'double');                                   %Sends the new Torque Value to Arduino
        fwrite(bt,N2,'double');
        fwrite(bt,N3,'double');
    catch
        disp("Impossible to set shaping parameters for BTRL");
    end
end

function N1_Edit_Callback(hObject, eventdata, handles)
% hObject    handle to N1_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of N1_Edit as text
%        str2double(get(hObject,'String')) returns contents of N1_Edit as a double


% --- Executes during object creation, after setting all properties.
function N1_Edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to N1_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function N2_Edit_Callback(hObject, eventdata, handles)
% hObject    handle to N2_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of N2_Edit as text
%        str2double(get(hObject,'String')) returns contents of N2_Edit as a double


% --- Executes during object creation, after setting all properties.
function N2_Edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to N2_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function N3_Edit_Callback(hObject, eventdata, handles)
% hObject    handle to N3_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of N3_Edit as text
%        str2double(get(hObject,'String')) returns contents of N3_Edit as a double


% --- Executes during object creation, after setting all properties.
function N3_Edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to N3_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in R_Bs_Frq.
function R_Bs_Frq_Callback(hObject, eventdata, handles)
% hObject    handle to R_Bs_Frq (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global GUI_Variables;
bt = GUI_Variables.BT; 

disp('Right torque adaption')
if (bt.Status=="open")
    try
        fwrite(bt,'p'); %char(112)
    catch
    end
end

% --- Executes on button press in R_N3_Adj.
function R_N3_Adj_Callback(hObject, eventdata, handles)
% hObject    handle to R_N3_Adj (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global GUI_Variables;
bt = GUI_Variables.BT; 
count_speed=0;
if (bt.Status=="open")
    try
        fwrite(bt,'o'); %char(111)
        count_speed=GUI_Variables.RLCount;
        GUI_Variables.R_COUNT_SPEED=[GUI_Variables.R_COUNT_SPEED;[count_speed,2]];
    catch
    end
end

% --- Executes on button press in L_N3_Adj.
function L_N3_Adj_Callback(hObject, eventdata, handles)
% hObject    handle to L_N3_Adj (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global GUI_Variables;
bt = GUI_Variables.BT; 
count_speed=0;
if (bt.Status=="open")
    try
        fwrite(bt,'O'); %char(79)
        count_speed=GUI_Variables.RLCount;
        GUI_Variables.L_COUNT_SPEED=[GUI_Variables.L_COUNT_SPEED;[count_speed,2]];
    catch
    end
end

% --- Executes on button press in L_Bs_Frq.
function L_Bs_Frq_Callback(hObject, eventdata, handles)
% hObject    handle to L_Bs_Frq (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global GUI_Variables;
bt = GUI_Variables.BT; 

if (bt.Status=="open")
    try
        fwrite(bt,'P'); %char(80)
    catch
    end
end


function R_Send_KF_Edit_Callback(hObject, eventdata, handles)
% hObject    handle to R_Send_KF_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of R_Send_KF_Edit as text
%        str2double(get(hObject,'String')) returns contents of R_Send_KF_Edit as a double


% --- Executes during object creation, after setting all properties.
function R_Send_KF_Edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to R_Send_KF_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in R_Send_KF.
function R_Send_KF_Callback(hObject, eventdata, handles)
% hObject    handle to R_Send_KF (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
new_KF = str2double(get(handles.R_Send_KF_Edit,'String'));         %Gets the Value entered into the edit Box in the G

global GUI_Variables
state=GUI_Variables.state;
disp(state);

bt = GUI_Variables.BT; 

if (bt.Status=="open")
    try
        fwrite(bt,'-'); %send the character "-"
        fwrite(bt,new_KF,'double');
        % str=["Send new Right KF ", num2str(new_KF)];
        disp("Send new Right KF ");
        disp(new_KF);

    catch
        disp("Impossible to write on bt the new KF");
    end
end

% --- Executes on button press in R_Check_KF.
function rkf=R_Check_KF_Callback(hObject, eventdata, handles)
global GUI_Variables;
bt = GUI_Variables.BT; 
rkf=0;
if (bt.Status=="open")
    try

        fwrite(bt,'~'); %send the character "~"
        message = fgetl(bt);
        if (strcmp(get(handles.Start_Trial,'Enable'), 'on'))
            if message(1) == 83 && message(length(message)-1) == 90 && message(2) == '~'
                indexes = find(message==44);
                KF_RL = str2double(message((indexes(1)+1):(indexes(2)-1)));
                set(handles.R_Check_KF_Text,'String',KF_RL);
                disp("Right Current KF ");
                disp(KF_RL);
                rkf=KF_RL;
            end
            %Curr_KF = str2double(fgets(bt));                                              %Gets the Current Arduino Torque Setpoint

        end
    catch
        disp("Impossible to know KF");
        set(handles.R_Check_KF_Text,'String',"NaN");
    end
end

if (bt.Status=="closed") 
    disp("Impossible to know KF");
    set(handles.R_Check_KF_Text,'String',"NaN");
end

% --- Executes on button press in L_Check_FSR_Th.
function lfsr=L_Check_FSR_Th_Callback(hObject, eventdata, handles)
global GUI_Variables;
bt = GUI_Variables.BT; 
lfsr=0;
if (bt.Status=="open")
    fwrite(bt,char('Q')); %send the character "Q"
    if(strcmp(get(handles.Start_Trial,'Enable'), 'on') )
        message = fgetl(bt);
        if message(1) == 83 && message(length(message)-1) == 90 && message(2) == 'Q'
            indexes = find(message==44);
            FSR_thresh_LL = str2double(message((indexes(1)+1):(indexes(2)-1)));
            set(handles.L_Check_FSR_Text,'String',FSR_thresh_LL);
            disp("Left Current FSR th ");
            disp(FSR_thresh_LL);
            lfsr=FSR_thresh_LL;
        end

    end
end
if (bt.Status=="closed") 
    disp("Impossible to know FSR THs");
    set(handles.L_Check_FSR_Text,'String',"NaN");
    set(handles.R_Check_FSR_Text,'String',"NaN");
end


function L_Send_FSR_Edit_Callback(hObject, eventdata, handles)
% hObject    handle to L_Send_FSR_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of L_Send_FSR_Edit as text
%        str2double(get(hObject,'String')) returns contents of L_Send_FSR_Edit as a double


% --- Executes during object creation, after setting all properties.
function L_Send_FSR_Edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to L_Send_FSR_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in L_Send_FSR_Th.
function L_Send_FSR_Th_Callback(hObject, eventdata, handles)
% hObject    handle to L_Send_FSR_Th (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global GUI_Variables
bt = GUI_Variables.BT;


if (bt.Status=="open")
    try
        fwrite(bt,'R'); %char 35 -> #, 36 -> $, 74-> J
        LFSRTH = str2double(get(handles.L_Send_FSR_Edit,'String'));               %Gets the Value entered into the edit Box in the G
        fwrite(bt,LFSRTH,'double');                                   %Sends the new Torque Value to Arduino
    catch
        disp("Impossible to set FSR th parameters for Left");
    end
end

% --- Executes on button press in R_Check_FSR_Th.
function rfsr=R_Check_FSR_Th_Callback(hObject, eventdata, handles)
global GUI_Variables;
bt = GUI_Variables.BT; 
rfsr=0;
if (bt.Status=="open")
    try
        fwrite(bt,char('q')); %send the character "Q"
        message = fgetl(bt);
        if message(1) == 83 && message(length(message)-1) == 90 && message(2) == 'q'
            indexes = find(message==44);
            Curr_TH_R = str2double(message((indexes(1)+1):(indexes(2)-1)));
            set(handles.R_Check_FSR_Text,'String',Curr_TH_R);
            disp("Right Current FSR th ");
            disp(Curr_TH_R);
            rfsr=Curr_TH_R;
        end
        

        %rfsr=str2double(Curr_TH_R);
    catch
        disp("Impossible to know R FSR TH");
        set(handles.R_Check_FSR_Text,'String',"NaN");
    end
end

if (bt.Status=="closed") 
    disp("Impossible to know FSR THs");
    set(handles.L_Check_FSR_Text,'String',"NaN");
    set(handles.R_Check_FSR_Text,'String',"NaN");
end


function R_Send_FSR_Edit_Callback(hObject, eventdata, handles)
% hObject    handle to R_Send_FSR_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of R_Send_FSR_Edit as text
%        str2double(get(hObject,'String')) returns contents of R_Send_FSR_Edit as a double
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



% --- Executes during object creation, after setting all properties.
function R_Send_FSR_Edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to R_Send_FSR_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in R_Send_FSR_Th.
function R_Send_FSR_Th_Callback(hObject, eventdata, handles)
% hObject    handle to R_Send_FSR_Th (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global GUI_Variables
bt = GUI_Variables.BT;


if (bt.Status=="open")
    try
        fwrite(bt,'r'); %char 35 -> #, 36 -> $, 74-> J
        RFSRTH = str2double(get(handles.R_Send_FSR_Edit,'String'));               %Gets the Value entered into the edit Box in the G
        fwrite(bt,RFSRTH,'double');                                   %Sends the new Torque Value to Arduino
    catch
        disp("Impossible to set FSR th parameters for Right");
    end
end

% --- Executes on button press in Enable_Arduino_Trig.
function Enable_Arduino_Trig_Callback(hObject, eventdata, handles)
% hObject    handle to Enable_Arduino_Trig (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of Enable_Arduino_Trig



function L_Ki_Edit_Callback(hObject, eventdata, handles)
% hObject    handle to L_Ki_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of L_Ki_Edit as text
%        str2double(get(hObject,'String')) returns contents of L_Ki_Edit as a double


% --- Executes during object creation, after setting all properties.
function L_Ki_Edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to L_Ki_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in L_Set_Perc.
function L_Set_Perc_Callback(hObject, eventdata, handles)
% hObject    handle to L_Set_Perc (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global GUI_Variables
bt = GUI_Variables.BT;

if (bt.Status=="open")
    try
        fwrite(bt,'S');
        L_PERC = str2double(get(handles.L_Set_Perc_Edit,'String'));               %Gets the Value entered into the edit Box in the G
        fwrite(bt,L_PERC,'double');                                   %Sends the new Torque Value to Arduino
    catch
        disp("Impossible to set Left Perc parameter for Left");
    end
end



function L_Set_Perc_Edit_Callback(hObject, eventdata, handles)
% hObject    handle to L_Set_Perc_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of L_Set_Perc_Edit as text
%        str2double(get(hObject,'String')) returns contents of L_Set_Perc_Edit as a double


% --- Executes during object creation, after setting all properties.
function L_Set_Perc_Edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to L_Set_Perc_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in R_Set_Perc.
function R_Set_Perc_Callback(hObject, eventdata, handles)
% hObject    handle to R_Set_Perc (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global GUI_Variables
bt = GUI_Variables.BT;

if (bt.Status=="open")
    try
        fwrite(bt,'s');
        R_PERC = str2double(get(handles.R_Set_Perc_Edit,'String'));               %Gets the Value entered into the edit Box in the G
        fwrite(bt,R_PERC,'double');                                   %Sends the new Torque Value to Arduino
    catch
        disp("Impossible to set Left Perc parameter for Right");
    end
end


function R_Set_Perc_Edit_Callback(hObject, eventdata, handles)
% hObject    handle to R_Set_Perc_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of R_Set_Perc_Edit as text
%        str2double(get(hObject,'String')) returns contents of R_Set_Perc_Edit as a double


% --- Executes during object creation, after setting all properties.
function R_Set_Perc_Edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to R_Set_Perc_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in L_Stop_N3.
function L_Stop_N3_Callback(hObject, eventdata, handles)
% hObject    handle to L_Stop_N3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global GUI_Variables
bt = GUI_Variables.BT;
count_speed=0;
if (bt.Status=="open")
    try
        fwrite(bt,'T');
        count_speed=GUI_Variables.RLCount;
        GUI_Variables.L_COUNT_SPEED=[GUI_Variables.L_COUNT_SPEED;[count_speed,1]];

    catch
    end
end

% --- Executes on button press in L_Stop_Trq.
function L_Stop_Trq_Callback(hObject, eventdata, handles)
% hObject    handle to L_Stop_Trq (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global GUI_Variables
bt = GUI_Variables.BT;

if (bt.Status=="open")
    try
        fwrite(bt,'I');
    catch
    end
end

% --- Executes on button press in R_Stop_N3.
function R_Stop_N3_Callback(hObject, eventdata, handles)
% hObject    handle to R_Stop_N3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global GUI_Variables
bt = GUI_Variables.BT;
count_speed=0;
if (bt.Status=="open")
    try
        fwrite(bt,'t');
        count_speed=GUI_Variables.RLCount;
        GUI_Variables.R_COUNT_SPEED=[GUI_Variables.R_COUNT_SPEED;[count_speed,0]];

    catch
    end
end

% --- Executes on button press in R_Stop_Trq.
function R_Stop_Trq_Callback(hObject, eventdata, handles)
% hObject    handle to R_Stop_Trq (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global GUI_Variables
bt = GUI_Variables.BT;

if (bt.Status=="open")
    try
        fwrite(bt,'i');
    catch
    end
end

% --- Executes on button press in Save_EXP_Prm.
function Save_EXP_Prm_Callback(hObject, eventdata, handles)
% hObject    handle to Save_EXP_Prm (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global GUI_Variables
bt = GUI_Variables.BT;

if (bt.Status=="open")
    try
        fwrite(bt,'!');
        disp(" ");
        disp("Saving EXP parameters");
        disp(" ");
    catch
    end
end


% --- Executes on button press in Load_From_File.
function Load_From_File_Callback(hObject, eventdata, handles)
% hObject    handle to Load_From_File (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global GUI_Variables
bt=GUI_Variables.BT;
disp("Before Starting we check the connections");
set(handles.statusText,'String','Checking Connections');
pause(0.01)

disp("Check BT")
valBT=Check_Bluetooth_Callback(hObject, eventdata, handles);
set(handles.statusText,'String','Connections Checked');
pause(0.01)


if valBT==0
    set(handles.statusText,'String','Connect BT before');
    return
end

disp('Load from File ');
strfile=input('Which file do you want to upload: ','s');

if not(exist(strfile))
    fprintf("File does not exists\n");
    
else

    set(handles.statusText,'String','Uploading data from file, WAIT!');
    fprintf(['Uploading data from file ',strfile,'\n']);
    Af=importdata(strfile);

    Data=Af.data;
    ASTR=Af.textdata(:,1);


    for i=1:size(Af.data)

        if strcmp('N1',ASTR{i})
            n1=Data(i);
        end
        if strcmp('N2',ASTR{i})
            n2=Data(i);
        end
        if strcmp('N3',ASTR{i})
            n3=Data(i);
        end
        
        if strcmp('KF_LL',ASTR{i})
            lkf=Data(i);
        end
        if strcmp('KF_RL',ASTR{i})
            rkf=Data(i);
        end
        
        if strcmp('FSR_TH_LL',ASTR{i})
            lfsr=Data(i);
        end
        if strcmp('FSR_TH_RL',ASTR{i})
            rfsr=Data(i);
        end
        
        if strcmp('KP_L',ASTR{i})
            lkp=Data(i);
        end
        if strcmp('KD_L',ASTR{i})
            lkd=Data(i);
        end
        if strcmp('KI_L',ASTR{i})
            lki=Data(i);
        end
        
        if strcmp('KP_R',ASTR{i})
            rkp=Data(i);
        end
        if strcmp('KD_R',ASTR{i})
            rkd=Data(i);
        end
        if strcmp('KI_R',ASTR{i})
            rki=Data(i);
        end
    end

    set(handles.L_Send_KF_Edit,'String',lkf);
    L_Send_KF_Callback(hObject, eventdata, handles);
    pause(0.4);
    set(handles.R_Send_KF_Edit,'String',rkf);
    R_Send_KF_Callback(hObject, eventdata, handles);
    pause(0.4);

    set(handles.L_Kp_Edit,'String',lkp);
    set(handles.L_Kd_Edit,'String',lkd);
    set(handles.L_Ki_Edit,'String',lki);
    L_Set_PID_Callback(hObject, eventdata, handles);
    pause(0.8);

    set(handles.R_Kp_Edit,'String',rkp);
    set(handles.R_Kd_Edit,'String',rkd);
    set(handles.R_Ki_Edit,'String',rki);
    R_Set_PID_Callback(hObject, eventdata, handles);
    pause(0.8);

    set(handles.N1_Edit,'String',n1);
    set(handles.N2_Edit,'String',n2);
    set(handles.N3_Edit,'String',n3);
    Set_Smoothing_Callback(hObject, eventdata, handles);
    pause(0.8);

    set(handles.L_Send_FSR_Edit,'String',lfsr);
    L_Send_FSR_Th_Callback(hObject, eventdata, handles);
    pause(0.4);

    set(handles.R_Send_FSR_Edit,'String',rfsr);
    R_Send_FSR_Th_Callback(hObject, eventdata, handles);
    pause(0.4);

    flushinput(bt);
    pause(.5);
    flushoutput(bt);
    pause(.5);

    set(handles.statusText,'String','Data from file uploaded');
end %end file exists



% --- Executes on button press in L_InverseSign_RadioButton.
function L_InverseSign_RadioButton_Callback(hObject, eventdata, handles)
% hObject    handle to L_InverseSign_RadioButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of L_InverseSign_RadioButton
% Hint: get(hObject,'Value') returns toggle state of L_InverseSign_RadioButton

global GUI_Variables
bt = GUI_Variables.BT;

if (bt.Status=="open")
    try
        disp('Left Inverse Sign')
        LD=get(hObject,'Value');
        disp(LD);

        if LD
            %activate Decline
            fwrite(bt,'W'); 
        else
            %deactivate Decline
            fwrite(bt,'X'); 
        end
    catch
    end
end


% --- Executes on button press in R_InverseSign_RadioButton.
function R_InverseSign_RadioButton_Callback(hObject, eventdata, handles)
% hObject    handle to R_InverseSign_RadioButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of R_InverseSign_RadioButton


global GUI_Variables
bt = GUI_Variables.BT;

if (bt.Status=="open")
    try
        disp('Right Inverse Sign')
        RD=get(hObject,'Value');
        disp(RD);
        if RD
            %activate Decline
            fwrite(bt,'w');
        else
            %deactivate Decline
            fwrite(bt,'x');
        end
    catch
    end
end


% --- Executes on button press in R_Check_Gain.
function R_Check_Gain_Callback(hObject, eventdata, handles)
% hObject    handle to R_Check_Gain (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global GUI_Variables
bt = GUI_Variables.BT;

if(bt.Status=="open")
    fwrite(bt,']');
end

if(strcmp(get(handles.Start_Trial,'Enable'), 'on'))
    message = fgetl(bt);
    if message(1) == 83 && message(length(message)-1) == 90 && message(2) == ']'
        indexes = find(message==44);
        R_Gain= str2double(message((indexes(1)+1):(indexes(2)-1)));
        disp("Right New Gain Setpoint")
        disp(R_Gain)
        set(handles.R_Check_Gain_Text,'String',R_Gain);
    end

end

% --- Executes on button press in R_Set_Gain.
function R_Set_Gain_Callback(hObject, eventdata, handles)
% hObject    handle to R_Set_Gain (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global GUI_Variables
bt = GUI_Variables.BT;

if(bt.Status=="open")
    fwrite(bt,'[');
end

try
    %set(handles.plotBut,'Enable','off');
    R_New_Gain = str2double(get(handles.R_Set_Gain_Edit,'String'));               %Gets the Value entered into the edit Box in the G
    fwrite(bt,R_New_Gain,'double');
    disp('Send to arduino Right Prop Gain');
    disp(R_New_Gain);
catch
end



function R_Set_Gain_Edit_Callback(hObject, eventdata, handles)
% hObject    handle to R_Set_Gain_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of R_Set_Gain_Edit as text
%        str2double(get(hObject,'String')) returns contents of R_Set_Gain_Edit as a double


% --- Executes during object creation, after setting all properties.
function R_Set_Gain_Edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to R_Set_Gain_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in L_Check_Gain.
function L_Check_Gain_Callback(hObject, eventdata, handles)
% hObject    handle to L_Check_Gain (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global GUI_Variables
bt = GUI_Variables.BT;

try
    if(bt.Status=="open")
        fwrite(bt,'}');
    end

    if(strcmp(get(handles.Start_Trial,'Enable'), 'on'))
        message = fgetl(bt);
        if message(1) == 83 && message(length(message)-1) == 90 && message(2) == '}'
            indexes = find(message==44);
            L_Gain= str2double(message((indexes(1)+1):(indexes(2)-1)));
            disp("Left New Gain Setpoint")
            disp(L_Gain)
            set(handles.L_Check_Gain_Text,'String',L_Gain);
        end

    end
catch
end


% --- Executes on button press in L_Set_Gain.
function L_Set_Gain_Callback(hObject, eventdata, handles)
% hObject    handle to L_Set_Gain (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global GUI_Variables
bt = GUI_Variables.BT;

try
    if(bt.Status=="open")
        fwrite(bt,'{');
    end

    %set(handles.plotBut,'Enable','off');
    L_New_Gain = str2double(get(handles.L_Set_Gain_Edit,'String'));               %Gets the Value entered into the edit Box in the G
    fwrite(bt,L_New_Gain,'double');
catch
end

try
    axes(handles.PROP_FUNCTION_axes);
    x=0.4:0.01:1.2;
    if GUI_Variables.Prop_mode==1
        plot(x,GUI_Variables.Setpoint*(128.1*x.^2-50.82*x+22.06)/(128.1-50.82+22.06))
    elseif GUI_Variables.Prop_mode==2
        plot(x,GUI_Variables.Setpoint*(128.1*x.^2-50.82*x+22.06)/(128.1-50.82+22.06))
    else
        plot(x,x*0)
    end
catch
end





function L_Set_Gain_Edit_Callback(hObject, eventdata, handles)
% hObject    handle to L_Set_Gain_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of L_Set_Gain_Edit as text
%        str2double(get(hObject,'String')) returns contents of L_Set_Gain_Edit as a double


% --- Executes during object creation, after setting all properties.
function L_Set_Gain_Edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to L_Set_Gain_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in Activate_Balance.
function Activate_Balance_Callback(hObject, eventdata, handles)
% hObject    handle to Activate_Balance (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of Activate_Balance
global GUI_Variables
bt = GUI_Variables.BT;

% disp('r');
if (bt.Status=="open")
    %     disp('r');

    PC=get(hObject,'Value');
    % disp(PC);
    try

        if PC
            %activate prop control
            fwrite(bt,'+');
            disp('Activate Balance Ctrl');
        else
            %deactivate prop control
            fwrite(bt,'=');
            disp('Deactivate Balance Ctrl');
        end
    catch
    end
end


% --- Executes on button press in radiobutton12.
function radiobutton12_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton12


% --- Executes on button press in radiobutton11.
function radiobutton11_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton11


% --- Executes during object creation, after setting all properties.
function R_Torque_CreateFcn(hObject, eventdata, handles)
% hObject    handle to R_Torque (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes on button press in Activate_Balance.
function radiobutton13_Callback(hObject, eventdata, handles)
% hObject    handle to Activate_Balance (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of Activate_Balance


% --- Executes on button press in L_Auto_KF.
function L_Auto_KF_Callback(hObject, eventdata, handles)
% hObject    handle to L_Auto_KF (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of L_Auto_KF
global GUI_Variables
bt = GUI_Variables.BT;

% disp('r');
if (bt.Status=="open")
    %     disp('r');

    LKF=get(hObject,'Value');
    % disp(PC);
    try

        if LKF
            %activate prop control
            fwrite(bt,'.');
            disp('Activate Left Auto KF');
        else
            %deactivate prop control
            fwrite(bt,';');
            disp('Deactivate Left Auto KF');
        end
    catch
    end
end

% % --- Executes on button press in R_Auto_KF.
% function R_Auto_KF_Callback(hObject, eventdata, handles)
% % hObject    handle to R_Auto_KF (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    structure with handles and user data (see GUIDATA)
% 
% % Hint: get(hObject,'Value') returns toggle state of R_Auto_KF
% global GUI_Variables
% bt = GUI_Variables.BT;
% 
% % disp('r');
% if (bt.Status=="open")
% %     disp('r');
% 
% RKF=get(hObject,'Value');
% % disp(PC);
% try
%     
% if RKF
%     %activate prop control
%     fwrite(bt,'+'); 
%     disp('Activate Right Auto KF');
% else
%     %deactivate prop control
%     fwrite(bt,'='); 
%     disp('Deactivate Right Auto KF');
% end
%     catch
%     end
% end


% --- Executes on button press in Activate_Prop_Pivot.
function Activate_Prop_Pivot_Callback(hObject, eventdata, handles)
% hObject    handle to Activate_Prop_Pivot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of Activate_Prop_Pivot

global GUI_Variables
bt = GUI_Variables.BT;

% disp('r');
if (bt.Status=="open")
    %     disp('r');

    PP=get(hObject,'Value');
    % disp(PC);
    try

        if PP
            %activate prop control
            fwrite(bt,'#');
            disp('Activate Prop Pivot Ctrl');
            axes(handles.PROP_FUNCTION_axes);
            x=0.4:0.01:1.2;
            plot(x,GUI_Variables.Setpoint*(128.1*x.^2-50.82*x+22.06)/(128.1-50.82+22.06));
        else
            %deactivate prop control
            fwrite(bt,'^');
            disp('Deactivate Prop Pivot Ctrl');
            axes(handles.PROP_FUNCTION_axes);
            x=0.4:0.01:1.2;
            plot(x,x*0);
        end
    catch
    end
end


% % --- Executes on button press in Slow_0_Trq.
% function Slow_0_Trq_Callback(hObject, eventdata, handles)
% % hObject    handle to Slow_0_Trq (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    structure with handles and user data (see GUIDATA)
% 
% 
% % --- Executes on button press in Fast_0_Trq.
% function Fast_0_Trq_Callback(hObject, eventdata, handles)
% % hObject    handle to Fast_0_Trq (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    structure with handles and user data (see GUIDATA)
% 

% --- Executes on button press in Fast_0_Trq.
function Fast_0_Trq_Callback(hObject, eventdata, handles)
% hObject    handle to Fast_0_Trq (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% SEND 'F'
global GUI_Variables
bt = GUI_Variables.BT;

disp('goes to 0 Fast')

try
    if(bt.Status=="open")
        fwrite(bt,'F');
    end
    fwrite(bt,-1,'double');
    pause(0.2);
    if(bt.Status=="open")
        fwrite(bt,'f');
    end
    fwrite(bt,-1,'double');

catch
    
end


% --- Executes on button press in Slow_0_Trq.
function Slow_0_Trq_Callback(hObject, eventdata, handles)
% hObject    handle to Slow_0_Trq (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global GUI_Variables
bt = GUI_Variables.BT;


disp('goes to 0 Slow')

try
    if(bt.Status=="open")
        fwrite(bt,'F');
    end
    fwrite(bt,0,'double');
    pause(0.2);
    if(bt.Status=="open")
        fwrite(bt,'f');
    end
    fwrite(bt,0,'double');

catch
    
end


% --- Executes on button press in Take_Baseline.
function Take_Baseline_Callback(hObject, eventdata, handles)
% hObject    handle to Take_Baseline (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global GUI_Variables
bt = GUI_Variables.BT;

try
    if(bt.Status=="open")
        fwrite(bt,'b');
    end



catch
end


% --- Executes on button press in Check_Baseline.
function Check_Baseline_Callback(hObject, eventdata, handles)
% hObject    handle to Check_Baseline (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global GUI_Variables
bt = GUI_Variables.BT;

try
    if(bt.Status=="open")
        fwrite(bt,'B');
    end

    disp('Check Baseline');
    %   if(strcmp(get(handles.Start_Trial,'Enable'), 'on'))
    message = fgetl(bt);
    if message(1) == 83 && message(length(message)-1) == 90 && message(2) == 'B'
        indexes = find(message==44);
        GUI_Variables.basel= str2double(message((indexes(1)+1):(indexes(2)-1)));
        GUI_Variables.baser= str2double(message((indexes(2)+1):(indexes(3)-1)));
        disp(GUI_Variables.basel);
        disp(GUI_Variables.baser);
    end

    %   end

catch
end



function L_Setpoint_Dorsi_Edit_Callback(hObject, eventdata, handles)
% hObject    handle to L_Setpoint_Dorsi_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of L_Setpoint_Dorsi_Edit as text
%        str2double(get(hObject,'String')) returns contents of L_Setpoint_Dorsi_Edit as a double


% --- Executes during object creation, after setting all properties.
function L_Setpoint_Dorsi_Edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to L_Setpoint_Dorsi_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function R_Setpoint_Dorsi_Edit_Callback(hObject, eventdata, handles)
% hObject    handle to R_Setpoint_Dorsi_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of R_Setpoint_Dorsi_Edit as text
%        str2double(get(hObject,'String')) returns contents of R_Setpoint_Dorsi_Edit as a double


% --- Executes during object creation, after setting all properties.
function R_Setpoint_Dorsi_Edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to R_Setpoint_Dorsi_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function L_Zero_Modif_Edit_Callback(hObject, eventdata, handles)
% hObject    handle to L_Zero_Modif_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of L_Zero_Modif_Edit as text
%        str2double(get(hObject,'String')) returns contents of L_Zero_Modif_Edit as a double


% --- Executes during object creation, after setting all properties.
function L_Zero_Modif_Edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to L_Zero_Modif_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in L_Set_Zero_Modif.
function L_Set_Zero_Modif_Callback(hObject, eventdata, handles)
% hObject    handle to L_Set_Zero_Modif (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global GUI_Variables
bt = GUI_Variables.BT;

try
    if(bt.Status=="open")
        fwrite(bt,'W');
    end

    L_Zero_Modif = str2double(get(handles.L_Zero_Modif_Edit,'String'));               %Gets the Value entered into the edit Box in the G
    fwrite(bt,L_Zero_Modif,'double');
    disp(L_Zero_Modif);
catch
end


function R_Zero_Modif_Edit_Callback(hObject, eventdata, handles)
% hObject    handle to R_Zero_Modif_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of R_Zero_Modif_Edit as text
%        str2double(get(hObject,'String')) returns contents of R_Zero_Modif_Edit as a double


% --- Executes during object creation, after setting all properties.
function R_Zero_Modif_Edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to R_Zero_Modif_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in R_Set_Zero_Modif.
function R_Set_Zero_Modif_Callback(hObject, eventdata, handles)
% hObject    handle to R_Set_Zero_Modif (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global GUI_Variables
bt = GUI_Variables.BT;

try
    if(bt.Status=="open")
        fwrite(bt,'X');
    end

    R_Zero_Modif = str2double(get(handles.R_Zero_Modif_Edit,'String'));               %Gets the Value entered into the edit Box in the G
    fwrite(bt,R_Zero_Modif,'double');
    disp(R_Zero_Modif);
catch
end


% --- Executes on button press in Balance_Baseline.
function Balance_Baseline_Callback(hObject, eventdata, handles)
% hObject    handle to Balance_Baseline (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global GUI_Variables
bt = GUI_Variables.BT;

try
    if(bt.Status=="open")
        fwrite(bt,'&');
    end

    disp('Balance Baseline');
catch
end
