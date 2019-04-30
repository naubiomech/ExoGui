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

% Last Modified by GUIDE v2.5 15-Apr-2019 10:45:22

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
function A_EXO_s_OpeningFcn(hObject, ~, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to A_EXO_s (see VARARGIN)

    handles.output = hObject;

    BT_INDEX = 4;
    BT_NAMES={'Exo_Bluetooth_3','Capstone_Bluetooth_1', ...
              'Exo_Bluetooth_2','Exo_High_Power','Jacks_Bluetooth', 'Jasons_Bluetooth'};
    BT_NAME = BT_NAMES{BT_INDEX};
    fprintf("Connecting to %s\n", BT_NAME);
    bt = Bluetooth(BT_NAME,1,'UserData',0,'InputBufferSize',2048*16*50); %Creates Bluetooth Object
    bt.Timeout=2;
    str_uno=input('Would you use the arduino trigger? [y/n] ','s');

    if (strcmp(str_uno,'y'))
        if exist('Uno', 'var')
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

    GUI_Variables = struct('BT',bt,'IP','0.0.0.0','t',0,'Timer',NaN,'state',0,'RLTRQ',NaN(1,60000), ...
                           'LLTRQ',NaN(1,60000),'LLFSR',NaN(1,60000),'RLFSR',NaN(1,60000),...
                           'LLVOLT',NaN(1,60000),'RLVOLT',NaN(1,60000),'LLVOLT_H',NaN(1,60000),'RLVOLT_H',NaN(1,60000),'RLCount',1,'LLCount',1,...
                           'COUNT',0,'UNO',Uno,'flag_calib',0,'flag_start',0,'first_calib',0,...
                           'L_COUNT_SPEED',[],'R_COUNT_SPEED',[],...
                           'RLSET',NaN(1,60000),'LLSET',NaN(1,60000),'MEM',ones(1,3)*2,...
                           'SIG1',NaN(1,60000),'SIG2',NaN(1,60000),'SIG3',NaN(1,60000),'SIG4',NaN(1,60000),...
                           'BASEL',NaN(1,60000),'BASER',NaN(1,60000),'BASEL_BIOFB',NaN(1,60000),'basel',0,'baser',0,'basel_biofb',0,'counter',0,...
                           'flag_end_trial',0,'count_connection',0,'BT_connection',NaN(1,100),...
                           'L_Bal_dyn_Toe',20,'L_Bal_dyn_Heel',30,'L_Bal_steady_Toe',40,'L_Bal_steady_Heel',50,...
                           'R_Bal_dyn_Toe',20,'R_Bal_dyn_Heel',30,'R_Bal_steady_Toe',40,'R_Bal_steady_Heel',50,...
                           'L_BAL_DYN_TOE',20*ones(1,60000),'L_BAL_DYN_HEEL',30*ones(1,60000),'L_BAL_STEADY_TOE',40*ones(1,60000),'L_BAL_STEADY_HEEL',50*ones(1,60000),...
                           'R_BAL_DYN_TOE',20*ones(1,60000),'R_BAL_DYN_HEEL',30*ones(1,60000),'R_BAL_STEADY_TOE',40*ones(1,60000),'R_BAL_STEADY_HEEL',50*ones(1,60000),...
                           'PropOn',0,'SSID','No_ID','TimeStamp',' ');
    
    GUI_Variables = Reset_GUI_Variables(GUI_Variables);
    handles.GUI_Variables = GUI_Variables;
    guidata(hObject, handles);



% --- Outputs from this function are returned to the command line.
function varargout = A_EXO_s_OutputFcn(~, ~, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
    varargout{1} = handles.output;

% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, ~, handles) %#ok<*DEFNU>
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: delete(hObject) closes the figure
    try
        GUI_Variables = handles.GUI_Variables;
        bt = GUI_Variables.BT;
        try
            fclose(bt);
        catch
            disp('Bluetooth already closed');
        end
    catch E
        disp(E);
    end
    try
        delete(hObject);
    catch E 
        disp(E);
    end

% --- Executes on button press in Start_Trial.
function Start_Trial_Callback(hObject, eventdata, handles)
%Make a further check of the connection before starting

    GUI_Variables = handles.GUI_Variables;
    bt = GUI_Variables.BT;

    if(GUI_Variables.flag_end_trial==1)
        GUI_Variables.flag_end_trial=0;
    end

    flushinput(bt);
    flushoutput(bt);
    GUI_Variables = Reset_GUI_Variables(GUI_Variables);

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


    set(handles.Calibrate_FSR,'Enable','on');
    set(handles.Calibrate_Torque,'Enable','off');
    set(handles.Check_Memory,'Enable','off');
    set(handles.Clean_Memory,'Enable','off');
    set(handles.Start_Trial,'Enable','off');
    set(handles.End_Trial,'Enable','on');
    set(handles.ATP_Mode,'Enable','on');
    set(handles.Start_Timer,'Enable','on');
    set(handles.Prop_Ctrl_Panel,'visible','on'); 

    GUI_Variables.flag_start=1;

    pause(.01);
    if state == 1 && bt.Status == "open"
        fwrite(bt,char(69)); % Sends ASCII character 69 to the Arduino, which the Arduino will
    end
    pause(.001);
    
    stamp = fix(clock);                         %Get time stamp
    time = sprintf('%.0f.',stamp(4:end));       %Change to string and add . delimiter
    GUI_Variables.TimeStamp = time(1:end-1);    %Avoid last dash and store in structure
    GUI_Variables.BT_Was_Disconnected = 0;
    GUI_Variables.start_count = 0;
    set(handles.statusText,'String','Trial has been started');
    tic
    handles.GUI_Variables = GUI_Variables;
    guidata(hObject,handles);
    if state == 1 % both connected
        disp('both connected')
        while strcmp(get(handles.Start_Trial,'Enable'), 'off')
            handles=guidata(hObject);
            GUI_Variables = handles.GUI_Variables;
            GUI_Variables = Update_GUI(GUI_Variables, handles);
            GUI_Variables = accept_message(bt,handles, GUI_Variables);
            handles.GUI_Variables = GUI_Variables;
            guidata(hObject, handles);
            pause(0.01);
        end
    end
    guidata(hObject, handles);
    
function GUI_Variables = accept_message(bt, handles, GUI_Variables)
    while bt.BytesAvailable>0
        GUI_Variables = Receive_Data_Message(GUI_Variables, handles);
    end
    

function GUI_Variables = Update_GUI(GUI_Variables, handles)
    RLCount = GUI_Variables.RLCount;
    LLCount = GUI_Variables.LLCount;
    
    bt = GUI_Variables.BT;
    start_count = GUI_Variables.start_count;
    BT_Was_Disconnected = GUI_Variables.BT_Was_Disconnected;
    
    if GUI_Variables.flag_calib==1
        if GUI_Variables.first_calib==0
            start_count=clock;
            start_count=start_count(6);
            disp("Start FSR Calib")
            GUI_Variables.first_calib=1;
        end
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

    %---------------------Optimization-----------------------------------------

    try %Flush input buffer if approaching overflow
        if GUI_Variables.BT.BytesAvailable>0.9*GUI_Variables.BT.InputBufferSize
            flushinput(GUI_Variables.BT);
            fprintf('BT input buffer flushed to avoid overflow \n')
        end
    catch
    end

    %------------Bang-Bang-------------------------
    if ~GUI_Variables.PropOn
        try
            if GUI_Variables.t~=0 && GUI_Variables.t.Status == "open"
                if GUI_Variables.t.BytesAvailable>0
                    Setpoints = fread(GUI_Variables.t,GUI_Variables.t.BytesAvailable);
                    if char(Setpoints') == "done"
                        fwrite(GUI_Variables.BT,'F');
                        fwrite(GUI_Variables.BT,0,'double');
                        fwrite(GUI_Variables.BT,0,'double');
                        fwrite(GUI_Variables.BT,'f');
                        fwrite(GUI_Variables.BT,0,'double');
                        fwrite(GUI_Variables.BT,0,'double');
                        fwrite(GUI_Variables.BT,'h');           %Optimization done
                        set(handles.statusText,'String',"Optimization generation complete.")
                    else
                        if ~contains(char(Setpoints'),'_') %One parameter bang-bang optimization (sigmoid)
                            if GUI_Variables.BT.Status == "open"
                                Trq_Setpoint = str2double(char(Setpoints'));    %Torque setpoint
                                fwrite(GUI_Variables.BT,'F');                   %Left leg setpoints
                                fwrite(GUI_Variables.BT,Trq_Setpoint,'double'); %Plantarflexion
                                fwrite(GUI_Variables.BT,0,'double');            %Dorsiflexion
                                fwrite(GUI_Variables.BT,'f');                   %Right leg setpoints
                                fwrite(GUI_Variables.BT,Trq_Setpoint,'double'); %Plantarflexion
                                fwrite(GUI_Variables.BT,0,'double');            %Dorsiflexion
                                disp(['Sent Data: ',num2str(Trq_Setpoint)]);
                            end
                        else %Two parameter bang-bang optimization (zero-jerk spline)
                            if GUI_Variables.BT.Status == "open"
                                Char_Setpoints = strsplit(char(Setpoints'),'_');
                                Trq_Setpoint = str2double(Char_Setpoints{1});    %Torque Setpoint
                                RT_Setpoint = str2double(Char_Setpoints{2})/100; %Rise time %
                                fwrite(GUI_Variables.BT,'$');       %Two optimization values incoming
                                fwrite(GUI_Variables.BT,Trq_Setpoint,'double');  %Torque setpoint
                                fwrite(GUI_Variables.BT,RT_Setpoint,'double');   %Rise time %
                                disp(['Sent Data: ',num2str(Trq_Setpoint),' , ', num2str(RT_Setpoint)]);
                            end
                        end
                    end
                end
            end
        catch error
            disp(error)
            fprintf('Unable to apply optimization setpoints \n')
        end

        %------------Proportional----------------------
    elseif GUI_Variables.PropOn
        try
            if GUI_Variables.t~=0 && GUI_Variables.t.Status == "open"
                if GUI_Variables.t.BytesAvailable>0
                    Setpoints = fread(GUI_Variables.t,GUI_Variables.t.BytesAvailable);
                    if char(Setpoints') == "done"
                        fwrite(GUI_Variables.BT,'"');
                        fwrite(GUI_Variables.BT,0,'double');
                        fwrite(GUI_Variables.BT,'h');
                        set(handles.statusText,'String',"Optimization generation complete.")
                    else
                        if ~contains(char(Setpoints'),'_')
                            Trq_Setpoint = str2double(char(Setpoints'));  %Torque Setpoint
                            if GUI_Variables.BT.Status == "open"
                                fwrite(GUI_Variables.BT,'"');   
                                fwrite(GUI_Variables.BT,Trq_Setpoint,'double');
                                disp(['Sent Data: ',num2str(Trq_Setpoint)]);
                            end
                        else
                            warning('Two parameter optimization for proportional control is not possible.');
                        end
                    end
                end
            end
        catch error
            disp(error)
            fprintf('Unable to apply optimization setpoint \n')
        end

    end
    %---------------------AUTORECONNECT--------------------

    if strcmp(get(handles.BT_Text,'String'),'On')
        % disp('Activating the BT auto reconnection Matlab gui side')

        if(bt.bytesAvailable==0 && bt.status=="open" && GUI_Variables.flag_end_trial==0)
            bt_wait_time=toc;

            if (bt_wait_time>1)
                BT_Was_Disconnected=1;
                disp("NO DATA AVAILABLE!!!");
                pause(.000000001);

                disp('Close BT')
                try
                    fclose(bt);
                catch
                end
                while(bt.status=="closed")
                    pause(.5);
                    disp('Try Open BT')
                    try
                        fopen(bt);
                    catch
                    end
                end
                disp('Wait half second more')
                pause(0.5);
                disp('New Status')
                disp(bt.status);
                if(bt.status=="open")
                    fwrite(bt,char(69));
                    GUI_Variables.count_connection=GUI_Variables.count_connection+1;
                    GUI_Variables.BT_connection(GUI_Variables.count_connection)=RLCount;
                end
                toc
                RLCount=RLCount+round(toc*100);
                LLCount=RLCount;
                tic
                tic
            end
        end

    end
    %---------------------------------------------------

    if ((bt.bytesAvailable > 0))
        if(BT_Was_Disconnected)
            GUI_Variables.count_connection=GUI_Variables.count_connection+1;
            GUI_Variables.BT_connection(GUI_Variables.count_connection)=RLCount;
            BT_Was_Disconnected=0;
        end
        tic
        
        GUI_Variables.RLCount = RLCount;
        GUI_Variables.LLCount = LLCount;
        
        GUI_Variables = Receive_Data_Message(GUI_Variables, handles);
        
        RLCount = GUI_Variables.RLCount;
        LLCount = GUI_Variables.LLCount;
        
        
        draw_graphs(handles, GUI_Variables)
        
    end

    GUI_Variables.RLCount = RLCount;
    GUI_Variables.LLCount = LLCount;
    GUI_Variables.start_count = start_count;
    GUI_Variables.BT_Was_Disconnected = BT_Was_Disconnected;

    
function draw_graphs(handles, GUI_Variables)
    plots = {{GUI_Variables.RLTRQ, GUI_Variables.RLSET}, ...
             {GUI_Variables.RLFSR}, ...
             {GUI_Variables.RLVOLT,GUI_Variables.RLVOLT_H,GUI_Variables.BASER}, ...
             {GUI_Variables.RLVOLT, GUI_Variables.RLVOLT_H,...
              GUI_Variables.R_BAL_DYN_TOE,...
              GUI_Variables.R_BAL_STEADY_TOE,...
              GUI_Variables.RLVOLT_H,...
              GUI_Variables.R_BAL_DYN_HEEL,...
              GUI_Variables.R_BAL_STEADY_HEEL}, ...
             {GUI_Variables.LLTRQ, GUI_Variables.LLSET}, ...
             {GUI_Variables.LLFSR}, ...
             {GUI_Variables.LLVOLT,GUI_Variables.LLVOLT_H,GUI_Variables.BASEL}, ...
             {GUI_Variables.LLVOLT,GUI_Variables.LLVOLT_H,...
              GUI_Variables.L_BAL_DYN_TOE,...
              GUI_Variables.L_BAL_STEADY_TOE,...
              GUI_Variables.LLVOLT_H,...
              GUI_Variables.L_BAL_DYN_HEEL,...
              GUI_Variables.L_BAL_STEADY_HEEL}, ...
             {GUI_Variables.SIG1}, ...
             {GUI_Variables.SIG2}, ...
             {GUI_Variables.SIG3}, ...
             {GUI_Variables.SIG4}};

    titles = {"RL Torque","RL State","RL Force Toe and Heel","Right Balance", ...
              "LL Torque","LL State","LL Force Toe and Heel","Left Balance", ...
              "SIG1","SIG2","SIG3","SIG4"};
    
    RLCount = GUI_Variables.RLCount;
    
    whichPlotLeft = get(handles.Bottom_Graph,'Value');
    whichPlotRight = get(handles.Top_Graph,'Value');
    draw_graph(whichPlotLeft, plots, titles, handles.Bottom_Axes, RLCount);
    draw_graph(whichPlotRight, plots, titles, handles.Top_Axes, RLCount);
%YF    
    if (strcmp(get(handles.Activate_BioFeedback_Text,'String'),'On')==1)%if biofeedback is on
        draw_graph_BF(plots, RLCount);
    end
    drawnow nocallbacks;

%YF
function draw_graph_BF(plots, RLCount)
figure(1)

ax=axes('XLim',[-10 10],'YLim',[0,2]);
grid off;
ylabel('Ratio')

x1 = [0  , -2, 2];
y1 = [0,-0.1,-0.1];

g1 = hgtransform;
patch('XData',x1,'YData',y1,'FaceColor',[0.144 0.852 0.850],'FaceAlpha',0.3,'Parent',g1)

g2 = hgtransform;
patch('XData',x1,'YData',y1,'FaceColor',[0.988 0.093 0.156],'FaceAlpha',0.3,'Parent',g2)

plotData1 = plots{10};%left update
plotData3 = plots{9}; %right update
plotData5 = plots{11};%right score
plotData6 = plots{12};%left score

dataLength = max(1, RLCount-1000):RLCount-1;
data1 = cellfun(@(x) x(dataLength), plotData1', 'UniformOutput', false);
data1 = cat(1,data1{:});
data3 = cellfun(@(x) x(dataLength), plotData3', 'UniformOutput', false);
data3 = cat(1,data3{:});
data5 = cellfun(@(x) x(dataLength), plotData5', 'UniformOutput', false);
data5 = cat(1,data5{:});
data6 = cellfun(@(x) x(dataLength), plotData6', 'UniformOutput', false);
data6 = cat(1,data6{:});

g1.Matrix=makehgtform('translate',[-5, data1(end), 0]);
g2.Matrix=makehgtform('translate',[5, data3(end), 0]);

hold on
plot([0 0],[0,2],'Linewidth',2,'Color','black')

%left side text
if data1(end)<1
    plot([-10 0],[1,1],'Linewidth',8,'Color','red')
    text(-9,1.8,['Left score: ' num2str(data6(end))],'fontsize',40);
    patch([-10 0 0 -10],[0 0 data1(end) data1(end)],'red','FaceAlpha',0.1)
else
    plot([-10 0],[1,1],'Linewidth',8,'Color','green')
    patch([-10 0 0 -10],[0 0 data1(end) data1(end)],'green','FaceAlpha',0.1)
    text(-9,1.8,['Left score: ' num2str(data6(end))],'fontsize',40);
    [y,Fs]=audioread('dragon-coin.wav');
    sound(y,Fs)
end

%right side text
if data3(end)<1
    plot([0 10],[1,1],'Linewidth',8,'Color','red')
    text(1,1.8,['Right score: ' num2str(data5(end))],'fontsize',40);
    patch([0 10 10 0],[0 0 data3(end) data3(end)],'red','FaceAlpha',0.1)
else
    plot([0 10],[1,1],'Linewidth',8,'Color','green')
    patch([0 10 10 0],[0 0 data3(end) data3(end)],'green','FaceAlpha',0.1)
    text(1,1.8,['Right score: ' num2str(data5(end))],'fontsize',40);
    [y,Fs]=audioread('dragon-coin.wav');
    sound(y,Fs)
end

    
function draw_graph(whichPlot, plots, titles, axis, RLCount)
    axes(axis);
    plotData = plots{whichPlot};
    plotTitle = titles{whichPlot};

    dataLength = max(1, RLCount-1000):RLCount-1;
    data = cellfun(@(x) x(dataLength), plotData', 'UniformOutput', false);
    data = cat(1,data{:});
    
    plot(dataLength, data);
    
    xlim([dataLength(1),RLCount]);
    title(plotTitle);
    
function GUI_Variables = Reset_GUI_Variables(GUI_Variables)
    allocated = 100000;
    
    GUI_Variables.basel = 0;
    GUI_Variables.baser = 0;

    GUI_Variables.RLCount = 2;
    GUI_Variables.LLCount = 2;
    GUI_Variables.LLFSR = NaN(1,allocated);
    GUI_Variables.RLFSR = NaN(1,allocated);
    GUI_Variables.RLTRQ = NaN(1,allocated);
    GUI_Variables.LLTRQ = NaN(1,allocated);

    GUI_Variables.RLSET = NaN(1,allocated);
    GUI_Variables.LLSET = NaN(1,allocated);

    GUI_Variables.LLVOLT = NaN(1,allocated);
    GUI_Variables.RLVOLT = NaN(1,allocated);

    GUI_Variables.LLVOLT_H = NaN(1,allocated);
    GUI_Variables.RLVOLT_H = NaN(1,allocated);


    GUI_Variables.L_BAL_DYN_HEEL=NaN(1,allocated);
    GUI_Variables.L_BAL_STEADY_HEEL=NaN(1,allocated);
    GUI_Variables.R_BAL_DYN_HEEL=NaN(1,allocated);
    GUI_Variables.R_BAL_STEADY_HEEL=NaN(1,allocated);

    GUI_Variables.L_BAL_DYN_TOE=NaN(1,allocated);
    GUI_Variables.L_BAL_STEADY_TOE=NaN(1,allocated);
    GUI_Variables.R_BAL_DYN_TOE=NaN(1,allocated);
    GUI_Variables.R_BAL_STEADY_TOE=NaN(1,allocated);

    GUI_Variables.SIG1=NaN(1,allocated);
    GUI_Variables.SIG2=NaN(1,allocated);
    GUI_Variables.SIG3=NaN(1,allocated);
    GUI_Variables.SIG4=NaN(1,allocated);

    GUI_Variables.BASEL=NaN(1,allocated);
    GUI_Variables.BASER=NaN(1,allocated);

    GUI_Variables.BASEL_BIOFB=NaN(1,allocated);
    
    GUI_Variables.BT_Was_Disconnected = 0;
    GUI_Variables.counter=0;
    GUI_Variables.COUNT = 0;

    GUI_Variables.flag_calib=0;
    GUI_Variables.flag_start=0;
    GUI_Variables.first_calib=0;
    GUI_Variables.flag_end_trial=1;

    GUI_Variables.L_COUNT_SPEED=[];
    GUI_Variables.R_COUNT_SPEED=[];



% --- Executes on button press in End_Trial.
function End_Trial_Callback(hObject, eventdata, handles)
    disp('')
    disp('End Trial button pushed')
    disp('')

    GUI_Variables = handles.GUI_Variables;
    bt = GUI_Variables.BT;

    if bt.Status=="open"
        LLTRQ = GUI_Variables.LLTRQ;
        LLFSR = GUI_Variables.LLFSR;
        RLTRQ = GUI_Variables.RLTRQ;
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

        BASEL_BIOFB=GUI_Variables.BASEL_BIOFB;


        L_BAL_DYN_HEEL=GUI_Variables.L_BAL_DYN_HEEL;
        L_BAL_STEADY_HEEL=GUI_Variables.L_BAL_STEADY_HEEL;
        L_BAL_DYN_TOE=GUI_Variables.L_BAL_DYN_TOE;
        L_BAL_STEADY_TOE=GUI_Variables.L_BAL_STEADY_TOE;

        R_BAL_DYN_HEEL=GUI_Variables.R_BAL_DYN_HEEL;
        R_BAL_STEADY_HEEL=GUI_Variables.R_BAL_STEADY_HEEL;
        R_BAL_DYN_TOE=GUI_Variables.R_BAL_DYN_TOE;
        R_BAL_STEADY_TOE=GUI_Variables.R_BAL_STEADY_TOE;


        disp("");
        disp(" I am going to terminate the system, my current state is: ");
        disp(state);
        disp("");

        if state == 1
            try
                fwrite(bt,char(71)); % Recognizes as stopping the motor and stop sending torque values
            catch
            end
        end

        try % if you turn off the motor before I push end trail this shouldn't be executed
            while((bt.bytesAvailable > 0))% I use the double while loop so I can incorporate a pause without having to pause every loop
                                          %I can pause after it think its finished looping (emptied the buffer), and then check if it should have finished looping (emptied the buffer) and if it is not finished, have it loop again
                while((bt.bytesAvailable > 0)) % While there are still torque values in the bluetooth buffer
                    if(bt.bytesAvailable > 0)
                        GUI_Variables = Receive_Data_Message(GUI_Variables,handles);
                    end                                                                  %Pause Long enough for any data in transit
                end
                pause(.5);
            end

        catch
        end


        LLTRQ = LLTRQ(1:(LLCount-1));
        RLTRQ = RLTRQ(1:(RLCount-1));
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

        BASEL_BIOFB=BASEL_BIOFB(1:(RLCount-1));

        L_BAL_DYN_HEEL=L_BAL_DYN_HEEL(1:(RLCount-1));
        L_BAL_STEADY_HEEL=L_BAL_STEADY_HEEL(1:(RLCount-1));
        L_BAL_DYN_TOE=L_BAL_DYN_TOE(1:(RLCount-1));
        L_BAL_STEADY_TOE=L_BAL_STEADY_TOE(1:(RLCount-1));

        R_BAL_DYN_HEEL=R_BAL_DYN_HEEL(1:(RLCount-1));
        R_BAL_STEADY_HEEL=R_BAL_STEADY_HEEL(1:(RLCount-1));
        R_BAL_DYN_TOE=R_BAL_DYN_TOE(1:(RLCount-1));
        R_BAL_STEADY_TOE=R_BAL_STEADY_TOE(1:(RLCount-1));



        if(state ==1)  %If both are on
            lengthDif = length(RLTRQ) - length(LLTRQ);                   %When both are on there is a possibility that one arduino sent more datapoints than the other
            disp(" ");
            disp(" Length DIF in state 1 : ");
            disp(lengthDif); % This would be caused by them not starting or stopping at the exact same time (to the nearest 10 ms)
            disp(" ");
            if lengthDif > 0 % It looks for the difference in legnth of datapoints to determine which has more datapoints
                addOn = NaN(1,lengthDif); % If the Left Or Right has more datapoints, it prepends the NaNs to start of the list with less datapoints
                LLTRQ = [addOn, LLTRQ];                                  %There is room for some error here, but the NaNs usually do not exceed 4, meaning that its at maximum,
                LLFSR = [addOn, LLFSR]; % 40 ms out of sync
                LLSET = [addOn,LLSET];
                LLVOLT = [addOn,LLVOLT];
                LLVOLT_H = [addOn,LLVOLT_H];
                SIG2 =[addOn,SIG2];


            end
            if lengthDif < 0
                lengthDif = -lengthDif;
                addOn = NaN(1,lengthDif);
                RLTRQ = [addOn,RLTRQ];
                RLFSR = [addOn,RLFSR];
                RLSET = [addOn,RLSET];
                RLVOLT = [addOn,RLVOLT];
                RLVOLT_H = [addOn,RLVOLT_H];
                SIG1=[addOn,SIG1];
                SIG3=[addOn,SIG3];
                SIG4=[addOn,SIG4];
                BASEL=[BASEL;addOn];
                BASER=[BASER;addOn];
                BASEL_BIOFB=[BASEL_BIOFB;addOn];
            end
            if lengthDif == 0 % If they are the same length, everything is dandy
            end
        end

        if (RLCount>=LLCount)
            TRIG=zeros(size(RLTRQ));
            L_SPEED=zeros(size(RLTRQ));
            R_SPEED=zeros(size(RLTRQ));
        else
            TRIG=zeros(size(LLTRQ));
            L_SPEED=zeros(size(LLTRQ));
            R_SPEED=zeros(size(LLTRQ));
        end

        dt = .01; % Since the Arduino is set to send values every 10 ms, dt is .01 S
        t = 1:length(RLTRQ);                                                %Creates a time Vector equal in length to the number of Torque Values Recieved
        t = t .* dt; % Scales the time Vector, knowing how often Arduino sends values,

        if isempty(GUI_Variables.COUNT) || (length(GUI_Variables.COUNT)==1) %beacuse the first is zero
        else
            count_trig_c=[1;GUI_Variables.COUNT(2:end);length(TRIG)];

            for i=2:length(count_trig_c)
                TRIG(count_trig_c(i-1):count_trig_c(i))=i - 2;
            end

        end

        if isempty(GUI_Variables.L_COUNT_SPEED)  %beacuse the first is zero
        else
            for i=1:size(GUI_Variables.L_COUNT_SPEED,1)
                L_SPEED(GUI_Variables.L_COUNT_SPEED(i,1))=GUI_Variables.L_COUNT_SPEED(i,2);
            end
        end

        if isempty(GUI_Variables.R_COUNT_SPEED)  %beacuse the first is zero
        else
            for i=1:size(GUI_Variables.R_COUNT_SPEED,1)
                R_SPEED(GUI_Variables.R_COUNT_SPEED(i,1))=GUI_Variables.R_COUNT_SPEED(i,2);
            end
        end

        A = {'t'; 'RLTRQ'; 'RLFSR'; 'RLSET'; 'RLVOLT'; 'RLVOLT_H'; 'LLTRQ';...
             'LLFSR'; 'LLSET'; 'LLVOLT'; 'LLVOLT_H'; ...
             'TRIG'; 'BASEL'; 'BASER' ; 'SIG1'; 'SIG2'; 'SIG3'; 'SIG4'; 'BSL_BIO'};
        A2={'L_BAL_DYN_HEEL';'L_BAL_STEADY_HEEL';'L_BAL_DYN_TOE';'L_BAL_STEADY_TOE';...
            'R_BAL_DYN_HEEL';'R_BAL_STEADY_HEEL';'R_BAL_DYN_TOE';'R_BAL_STEADY_TOE'};
        A_mat = [t; RLTRQ; RLFSR; RLSET; RLVOLT; RLVOLT_H; LLTRQ;...
                 LLFSR; LLSET; LLVOLT; LLVOLT_H; ...
                 TRIG; BASEL; BASER ; SIG1; SIG2; SIG3; SIG4; BASEL_BIOFB];                                 %Creates a vector that holds the time and data
                                                                                                            % A = [t; RLTRQ; RLSET; LLTRQ; LLSET]
        A_mat2=[L_BAL_DYN_HEEL;L_BAL_STEADY_HEEL;L_BAL_DYN_TOE;L_BAL_STEADY_TOE;...
                R_BAL_DYN_HEEL;R_BAL_STEADY_HEEL;R_BAL_DYN_TOE;R_BAL_STEADY_TOE];

        A=[A;A2];
        A_mat=[A_mat;A_mat2];

        
        currDir = cd;       % Current directory
        saveDir = [GUI_Variables.SSID,'_',date];
        savePath = [currDir,'\',saveDir];    % Save directory specific to subject and date
        if ~exist(saveDir, 'dir')
            mkdir(currDir, saveDir);           % Make a save directory
        end
        Filename = sprintf('%s_%d',fullfile(savePath,[GUI_Variables.SSID,'_',date,'_',GUI_Variables.TimeStamp,'_',...
            'Trial_Number_']),bt.UserData);               %Creates a new filename called "Torque_#"


        fileID = fopen(Filename,'w'); % Actually creates that file
        pause(.01);
        str_fileID='';
        str_fileID_title='\t';
        for jk=1:size(A,1)

            if jk==1
                str_fileID=[str_fileID,'\t','%0.2f\t\t '];
            elseif jk==size(A,1)
                str_fileID=[str_fileID,'%0.2f\n '];
            else
                str_fileID=[str_fileID,'%0.2f\t\t '];
            end

            if length(A{jk})>6
                str_fileID_title=[str_fileID_title,A{jk},'\t\t'];
            else
                str_fileID_title=[str_fileID_title,A{jk},'\t\t\t'];
            end

        end

        fprintf(fileID,[str_fileID_title,'\n']);
        fprintf(fileID,str_fileID,A_mat);

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

        try
            [n1,n2,n3]=Get_Smoothing_Callback(hObject, eventdata, handles);
            pause(0.5);
            lfsr=L_Check_FSR_Th_Callback(hObject, eventdata, handles);
            rfsr=R_Check_FSR_Th_Callback(hObject, eventdata, handles);
            lkf=L_Check_KF_Callback(hObject, eventdata, handles);
            rkf=R_Check_KF_Callback(hObject, eventdata, handles);
            [lkp,lkd,lki]=L_Get_PID_Callback(hObject, eventdata, handles);
            [rkp,rkd,rki]=R_Get_PID_Callback(hObject, eventdata, handles);
        catch

            n1 = -1;
            n2 = -1;
            n3 = -1;
            lfsr = -1;
            rfsr = -1;
            lkf = -1;
            rkf = -1;
            lkp = -1;
            lkd = -1;
            lki = -1;
            rkp = -1;
            rkd = -1;
            rki = -1;
        end

        currDir = cd;       % Current directory
        saveDir = [GUI_Variables.SSID,'_',date];
        savePath = [currDir,'\',saveDir];    % Save directory specific to subject and date
        if ~exist(saveDir, 'dir')
            mkdir(currDir, saveDir);           % Make a save directory
        end
        Filename = sprintf('%s_%d',fullfile(savePath,[GUI_Variables.SSID,'_',date,'_',...
            GUI_Variables.TimeStamp,'_','Parameters_Trial_Number_']),...
                           bt.UserData);               %Creates a new filename called "Torque_#"
        fileID = fopen(Filename,'w'); % Actually creates that file
        pause(.01);
        fprintf(fileID,['N1 = ',num2str(n1),'\n']);
        fprintf(fileID,['N2 = ',num2str(n2),'\n']);
        fprintf(fileID,['N3 = ',num2str(n3),'\n']);
        fprintf(fileID,['KF_LL = ',num2str(lkf),'\n']);
        fprintf(fileID,['KF_RL = ',num2str(rkf),'\n']);
        fprintf(fileID,['FSR_TH_LL = ',num2str(lfsr),'\n']);
        fprintf(fileID,['FSR_TH_RL = ',num2str(rfsr),'\n']);
        fprintf(fileID,['KP_L = ',num2str(lkp),'\n']);
        fprintf(fileID,['KD_L = ',num2str(lkd),'\n']);
        fprintf(fileID,['KI_L = ',num2str(lki),'\n']);
        fprintf(fileID,['KP_R = ',num2str(rkp),'\n']);
        fprintf(fileID,['KD_R = ',num2str(rkd),'\n']);
        fprintf(fileID,['KI_R = ',num2str(rki),'\n']);
        fclose(fileID);
        
%  TN 04-26-2019        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
        left_plant_peak_mean = 0; 
        right_plant_peak_mean = 0; 
        left_leg_Curr_Combined = 0;
        right_leg_Curr_Combined = 0;
        left_leg_fsr_Combined_peak_ref = 0;
        right_leg_fsr_Combined_peak_ref = 0;
        left_leg_fsr_Toe_peak_ref = 0;
        right_leg_fsr_Toe_peak_ref = 0;
        left_leg_fsr_Heel_peak_ref = 0;
        right_leg_fsr_Heel_peak_ref = 0;
        left_leg_torque_calibration_value = 0;
        right_leg_torque_calibration_value = 0;


    fwrite(bt,'e');
          message = fgetl(bt);
        if message(1) == 83 && message(length(message)-1) == 90 && message(2) == 'P'
            indexes = find(message==44);
        left_plant_peak_mean = str2double(message((indexes(1)+1):(indexes(2)-1)));
        right_plant_peak_mean = str2double(message((indexes(2)+1):(indexes(3)-1)));
        left_leg_Curr_Combined = str2double(message((indexes(3)+1):(indexes(4)-1)));
        right_leg_Curr_Combined = str2double(message((indexes(4)+1):(indexes(5)-1)));
        left_leg_fsr_Combined_peak_ref = str2double(message((indexes(5)+1):(indexes(6)-1)));
        right_leg_fsr_Combined_peak_ref = str2double(message((indexes(6)+1):(indexes(7)-1)));
        left_leg_fsr_Toe_peak_ref = str2double(message((indexes(7)+1):(indexes(8)-1)));
        right_leg_fsr_Toe_peak_ref = str2double(message((indexes(8)+1):(indexes(9)-1)));
        left_leg_fsr_Heel_peak_ref = str2double(message((indexes(9)+1):(indexes(10)-1)));
        right_leg_fsr_Heel_peak_ref = str2double(message((indexes(10)+1):(indexes(11)-1)));
        left_leg_torque_calibration_value = str2double(message((indexes(11)+1):(indexes(12)-1)));
        right_leg_torque_calibration_value = str2double(message((indexes(12)+1):(indexes(13)-1)));

        
        end
            
 %   currDir = cd;       % Current directory
        saveDir_P = [GUI_Variables.SSID,'_',date,'_Proportional_Parameters'];    % Save directory specific to subject and date
            savePath_P = [currDir,'\',saveDir_P];    % Save directory specific to subject and date
        if ~exist(saveDir_P, 'dir')
            mkdir(currDir, saveDir_P);           % Make a save directory
        end
        Filename_P = sprintf('%s_%d.txt',fullfile(savePath_P,[GUI_Variables.SSID,'_',date,'_',...
            GUI_Variables.TimeStamp,'_','Proportional_Parameters_']),bt.UserData);               %Creates a new filename called "Torque_#"
                                                                           %Where # is the trial number                                                                           
        fileID_P = fopen(Filename_P,'w');                                      %Actually creates that file
        pause(.01);
        fprintf(fileID_P,['left_plant_peak_mean = ', num2str(left_plant_peak_mean),'\r\n']);
        fprintf(fileID_P,['right_plant_peak_mean = ', num2str(right_plant_peak_mean),'\r\n']);
        fprintf(fileID_P,['left_leg_Curr_Combined = ', num2str(left_leg_Curr_Combined),'\r\n']);
        fprintf(fileID_P,['right_leg_Curr_Combined = ', num2str(right_leg_Curr_Combined),'\r\n']);
        fprintf(fileID_P,['left_leg_fsr_Combined_peak_ref = ', num2str(left_leg_fsr_Combined_peak_ref),'\r\n']);
        fprintf(fileID_P,['right_leg_fsr_Combined_peak_ref = ', num2str(right_leg_fsr_Combined_peak_ref),'\r\n']);
        fprintf(fileID_P,['left_leg_fsr_Toe_peak_ref = ', num2str(left_leg_fsr_Toe_peak_ref),'\r\n']);
        fprintf(fileID_P,['right_leg_fsr_Toe_peak_ref = ', num2str(right_leg_fsr_Toe_peak_ref),'\r\n']);
        fprintf(fileID_P,['left_leg_fsr_Heel_peak_ref = ', num2str(left_leg_fsr_Heel_peak_ref),'\r\n']);
        fprintf(fileID_P,['right_leg_fsr_Heel_peak_ref = ', num2str(right_leg_fsr_Heel_peak_ref),'\r\n']);
        fprintf(fileID_P,['left_leg_torque_calibration_value = ', num2str(left_leg_torque_calibration_value),'\r\n']);
        fprintf(fileID_P,['right_leg_torque_calibration_value = ', num2str(right_leg_torque_calibration_value),'\r\n']);
        fclose(fileID_P);
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        set(handles.statusText,'String','Data has finished being Saved');
        GUI_Variables = Reset_GUI_Variables(GUI_Variables);
        bt.UserData = bt.UserData + 1; % Increments the trial number

        Reset_Timer_Callback(0, 0, handles);
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
        set(handles.End_Trial,'Enable','off'); % Disables the button to stop the trial
        set(handles.Start_Trial,'Enable','on');
        set(handles.Activate_BioFeedback_Text,'String','Off');
        GUI_Variables.counter=0;
        set(handles.TRIG_NUM_TEXT,'String',0);
        set(handles.Start_Timer,'enable','Off');
%        set(handles.Activate_Prop_Pivot,'value',0);
%        set(handles.Activate_Prop_ID,'value',0);
%         set(handles.Activate_Prop_Pivot,'enable','on');
%         set(handles.Activate_Prop_ID,'enable','on');
%        set(handles.Activate_Prop_Ctrl,'string','Activate Prop Control');
%        set(handles.Prop_Ctrl_Panel,'visible','off');
        


    else

        Reset_Timer_Callback(0, 0, handles);
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
        set(handles.Activate_BioFeedback_Text,'String','Off');
        disp("System not connected");
        set(handles.statusText,'String','System not connected');
        set(handles.Start_Timer,'enable','Off');
%        set(handles.Activate_Prop_Pivot,'value',0);
%        set(handles.Activate_Prop_ID,'value',0);
%         set(handles.Activate_Prop_Pivot,'enable','off');
%         set(handles.Activate_Prop_ID,'enable','off');
%        set(handles.Activate_Prop_Ctrl,'string','Activate Prop Control');
%        set(handles.Prop_Ctrl_Panel,'visible','off');
 
    end
    
    handles.GUI_Variables = GUI_Variables;
    guidata(hObject, handles);



% --- Executes on button press in Calibrate_Torque.
function Calibrate_Torque_Callback(~, ~, handles)
% hObject    handle to Calibrate_Torque (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    set(handles.statusText,'String',"Calibrating the Torque Sensors! <(^_^<)");
    pause(.01);
    GUI_Variables = handles.GUI_Variables;
    bt = GUI_Variables.BT;
    if(bt.Status == "open")
        try
            fwrite(bt,char(72));
        catch
        end
    end
    pause(2);
    set(handles.statusText,'String',"The Torque Sensors have been Calibrated! (>^_^)>");

% --- Executes on button press in Calibrate_FSR.
function Calibrate_FSR_Callback(hObject, ~, handles)
% hObject    handle to Calibrate_FSR (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    GUI_Variables = handles.GUI_Variables;

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
    handles.GUI_Variables = GUI_Variables;
    guidata(hObject, handles);


% --- Executes on button press in Check_Memory.
function Check_Memory_Callback(hObject, ~, handles)
    disp("Checking Memory Status")
    set(handles.statusText,'String','Checking Memory Status');
    pause(0.01);

    GUI_Variables = handles.GUI_Variables;
    bt = GUI_Variables.BT;
    mem=GUI_Variables.MEM;
    if(bt.Status == "open")
        fwrite(bt,char(60));
        GUI_Variables = Receive_Data_Message(GUI_Variables, handles);
    else
        disp("the status bt is not opened")
        set(handles.axes8,'Color',[0 0 0])
        set(handles.axes10,'Color',[0 0 0])
        set(handles.EXP_Params_axes,'Color',[0 0 0])
        mem=ones(1,3)*2;
    end
    GUI_Variables.MEM=mem;
    set(handles.statusText,'String','Memory Status Checked');
    handles.GUI_Variables = GUI_Variables;
    guidata(hObject, handles);

% --- Executes on button press in Clean_Memory.
function Clean_Memory_Callback(~, ~, handles)
% hObject    handle to Clean_Memory (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    GUI_Variables = handles.GUI_Variables;
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
function lkf=L_Check_KF_Callback(hObject, ~, handles)
    GUI_Variables = handles.GUI_Variables;
    bt = GUI_Variables.BT;
    lkf=0;
    if (bt.Status=="open")
        fwrite(bt,'`'); % send the character "`"
        if (strcmp(get(handles.Start_Trial,'Enable'), 'on'))
            GUI_Variables = Receive_Data_Message(GUI_Variables,handles);
        end
    end

    if (bt.Status=="closed")
        disp("Impossible to know KF");
        set(handles.L_Check_KF_Text,'String',"NaN");
    end
    handles.GUI_Variables = GUI_Variables;
    guidata(hObject, handles);

% --- Executes on button press in L_Send_KF.
function L_Send_KF_Callback(hObject, ~, handles)
% hObject    handle to L_Send_KF (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

    new_KF = str2double(get(handles.L_Send_KF_Edit,'String'));         %Gets the Value entered into the edit Box in the G

    GUI_Variables = handles.GUI_Variables;

    bt = GUI_Variables.BT;

    if (bt.Status=="open")
        try
            fwrite(bt,char(95)); % send the character "_"
            fwrite(bt,new_KF,'double');
            disp("Send new Left KF ");
            disp(new_KF);
            L_Check_KF_Callback(hObject,0,handles);
        catch 
            disp("Impossible to write on bt the new KF");
        end
    end


function L_Send_KF_Edit_Callback(~, ~, ~)
% hObject    handle to L_Send_KF_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of L_Send_KF_Edit as text
%        str2double(get(hObject,'String')) returns contents of L_Send_KF_Edit as a double


% --- Executes during object creation, after setting all properties.
function L_Send_KF_Edit_CreateFcn(hObject, ~, ~)
% hObject    handle to L_Send_KF_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end


% --- Executes on selection change in Top_Graph.
function Top_Graph_Callback(~, ~, ~)
% hObject    handle to Top_Graph (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns Top_Graph contents as cell array
%        contents{get(hObject,'Value')} returns selected item from Top_Graph


% --- Executes during object creation, after setting all properties.
function Top_Graph_CreateFcn(hObject, ~, ~)
% hObject    handle to Top_Graph (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end


% --- Executes on selection change in Bottom_Graph.
function Bottom_Graph_Callback(~, ~, ~)
% hObject    handle to Bottom_Graph (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns Bottom_Graph contents as cell array
%        contents{get(hObject,'Value')} returns selected item from Bottom_Graph


% --- Executes during object creation, after setting all properties.
function Bottom_Graph_CreateFcn(hObject, ~, ~)
% hObject    handle to Bottom_Graph (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end


% --- Executes on button press in Send_Trig.
function Send_Trig_Callback(hObject, ~, handles)
% hObject    handle to Send_Trig (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% tic
    GUI_Variables = handles.GUI_Variables;
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
    

    GUI_Variables.COUNT=[GUI_Variables.COUNT;max(1,count_trig-50)];
    GUI_Variables.counter=GUI_Variables.counter+1;
    cane=GUI_Variables.counter;
    set(handles.TRIG_NUM_TEXT,'String',num2str(cane));
    try
        for i = 1:GUI_Variables.counter
            writeDigitalPin(GUI_Variables.UNO, 'D5', 1);
            pause(0.01);
            writeDigitalPin(GUI_Variables.UNO, 'D5', 0);
            pause(0.01);
        end
    catch
    end
    handles.GUI_Variables = GUI_Variables;
    guidata(hObject, handles);

% --- Executes on button press in Check_Bluetooth.
function valBT=Check_Bluetooth_Callback(hObject, ~, handles)
    GUI_Variables = handles.GUI_Variables;
    bt = GUI_Variables.BT;
    pause(.01);
    set(handles.statusText,'String',"Checking Bluetooth Connection");
    pause(.01);
    draw_graphs(handles, GUI_Variables);   
    try
        fwrite(bt,char(78))
        try
            GUI_Variables = Receive_Data_Message(GUI_Variables, handles);
        catch
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
    handles.GUI_Variables = GUI_Variables;
    guidata(hObject, handles);

% --- Executes on button press in Connect_BT.
function Connect_BT_Callback(hObject, ~, handles)
% hObject    handle to Connect_BT (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    GUI_Variables = handles.GUI_Variables;
    bt = GUI_Variables.BT;
    fprintf('Attempting to make a connection to the bluetooth\n');
    set(handles.statusText,'String',"Attempting to make a Connection to the Bluetooth!");
    pause(.01);


    try
        fopen(bt); % Attempts to Make a connection to Bluetooth Object
    catch
        set(handles.flag_bluetooth,'Color',[1 0 0]);
        set(handles.axes8,'Color',[0 0 0])
        set(handles.axes10,'Color',[0 0 0])
        set(handles.EXP_Params_axes,'Color',[0 0 0])
    end % Makes a connection to Bluetooth Object

    if(bt.status == "open")
        set(handles.flag_bluetooth,'Color',[0 1 0]);
        set(handles.axes8,'Color',[0 0 1])
        set(handles.axes10,'Color',[0 0 1])
        set(handles.EXP_Params_axes,'Color',[0 0 1])
        fprintf("Made a connection to the Right Ankle bluetooth!\n");
        set(handles.statusText,'String',"Made a Connection to the Right Ankle Bluetooth!");
        pause(1);
        Version_Button_Callback(hObject,' ',handles);
    end

    if(bt.status == "closed")
        set(handles.flag_bluetooth,'Color',[1 0 0]);
        set(handles.axes8,'Color',[0 0 0])
        set(handles.axes10,'Color',[0 0 0])
        set(handles.EXP_Params_axes,'Color',[0 0 0])
        set(handles.statusText,'String',"Could Not Connect to the Right Ankle Bluetooth :(  Try Again! (If it fails 3+ times attempt a power cycle)");
    end
    handles.GUI_Variables = GUI_Variables;
    guidata(hObject, handles);


function R_Ki_Edit_Callback(~, ~, ~)
% hObject    handle to R_Ki_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of R_Ki_Edit as text
%        str2double(get(hObject,'String')) returns contents of R_Ki_Edit as a double


% --- Executes during object creation, after setting all properties.
function R_Ki_Edit_CreateFcn(hObject, ~, ~)
% hObject    handle to R_Ki_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end



function R_Kd_Edit_Callback(~, ~, ~)
% hObject    handle to R_Kd_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of R_Kd_Edit as text
%        str2double(get(hObject,'String')) returns contents of R_Kd_Edit as a double


% --- Executes during object creation, after setting all properties.
function R_Kd_Edit_CreateFcn(hObject, ~, ~)
% hObject    handle to R_Kd_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end



function R_Kp_Edit_Callback(~, ~, ~)
% hObject    handle to R_Kp_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of R_Kp_Edit as text
%        str2double(get(hObject,'String')) returns contents of R_Kp_Edit as a double


% --- Executes during object creation, after setting all properties.
function R_Kp_Edit_CreateFcn(hObject, ~, ~)
% hObject    handle to R_Kp_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end


% --- Executes on button press in R_Set_PID.
function R_Set_PID_Callback(~, ~, handles)
% hObject    handle to R_Set_PID (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% SEND 'm'
    GUI_Variables = handles.GUI_Variables;
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
        pause(0.3);
        R_Get_PID_Callback(0,0,handles);
    end

% --- Executes on button press in R_Get_PID.
function [rkp,rkd,rki]=R_Get_PID_Callback(~, ~, handles)
% SEND 'k' char 107
    GUI_Variables = handles.GUI_Variables;
    bt = GUI_Variables.BT;

    if(bt.Status=="open")
        fwrite(bt,char(107));
        if(strcmp(get(handles.Start_Trial,'Enable'), 'on') )
            Receive_Data_Message(GUI_Variables, handles);
        end
    end
    rkp = get(handles.R_Kp_text,'String');
    rkd = get(handles.R_Kp_text,'String');
    rki = get(handles.R_Kp_text,'String');

% --- Executes on button press in L_Get_PID.
function [lkp,lkd,lki]=L_Get_PID_Callback(~, ~, handles)
% SEND 'K' char 75
    GUI_Variables = handles.GUI_Variables;
    bt = GUI_Variables.BT;

    if(bt.Status=="open")
        fwrite(bt,char(75));
        if(strcmp(get(handles.Start_Trial,'Enable'), 'on'))
            Receive_Data_Message(GUI_Variables, handles);
        end
    end
    
    lkp = get(handles.L_Kp_text,'String');
    lkd = get(handles.L_Kd_text,'String');
    lki = get(handles.L_Ki_text,'String');
    


% --- Executes on button press in L_Set_PID.
function L_Set_PID_Callback(~, ~, handles)
% hObject    handle to L_Set_PID (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% SEND 'M'
    GUI_Variables = handles.GUI_Variables;
    bt = GUI_Variables.BT;
    if(bt.Status=="open")
        fwrite(bt,'M');
        kp = str2double(get(handles.L_Kp_Edit,'String')); % Gets the Value entered into the edit Box in the G
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


        fwrite(bt,kp,'double'); % Sends the new Torque Value to Arduino
        fwrite(bt,kd,'double');
        fwrite(bt,ki,'double');
        pause(0.3);
        L_Get_PID_Callback(0,0,handles);
    end

function L_Kp_Edit_Callback(~, ~, ~)
% hObject    handle to L_Kp_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of L_Kp_Edit as text
%        str2double(get(hObject,'String')) returns contents of L_Kp_Edit as a double


% --- Executes during object creation, after setting all properties.
function L_Kp_Edit_CreateFcn(hObject, ~, ~)
% hObject    handle to L_Kp_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end



function L_Kd_Edit_Callback(~, ~, ~)
% hObject    handle to L_Kd_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of L_Kd_Edit as text
%        str2double(get(hObject,'String')) returns contents of L_Kd_Edit as a double


% --- Executes during object creation, after setting all properties.
function L_Kd_Edit_CreateFcn(hObject, ~, ~)
% hObject    handle to L_Kd_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end


% --- Executes on button press in R_Set_Setpoint.
function R_Set_Setpoint_Callback(hObject, ~, handles)
% hObject    handle to R_Set_Setpoint (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% SEND f
    GUI_Variables = handles.GUI_Variables;
    bt = GUI_Variables.BT;

    if(bt.Status=="open")
        fwrite(bt,char(102));
    end

    NewSetpoint = str2double(get(handles.R_Setpoint_Edit,'String')); % Gets the Value entered into the edit Box in the G
    fwrite(bt,NewSetpoint,'double');
    NewSetpoint_Dorsi = str2double(get(handles.R_Setpoint_Dorsi_Edit, 'String')); % Gets the Value entered into the edit Box in the G
    fwrite(bt,NewSetpoint_Dorsi,'double');

    pause(0.3);
    R_Get_Setpoint_Callback(hObject,0,handles);

function R_Setpoint_Edit_Callback(~, ~, ~)
% hObject    handle to R_Setpoint_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of R_Setpoint_Edit as text
%        str2double(get(hObject,'String')) returns contents of R_Setpoint_Edit as a double


% --- Executes during object creation, after setting all properties.
function R_Setpoint_Edit_CreateFcn(hObject, ~, ~)
% hObject    handle to R_Setpoint_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end


% --- Executes on button press in R_Get_Setpoint.
function R_Get_Setpoint_Callback(hObject, ~, handles)
%  SEND 'd'
    GUI_Variables = handles.GUI_Variables;
    bt = GUI_Variables.BT;

    if(bt.Status=="open")
        fwrite(bt,char(100));
    end

    if(strcmp(get(handles.Start_Trial,'Enable'), 'on'))
        GUI_Variables = Receive_Data_Message(GUI_Variables,handles);
    end
    handles.GUI_Variables = GUI_Variables;
    guidata(hObject, handles);
    
% --- Executes on button press in L_Get_Setpoint.
function L_Get_Setpoint_Callback(hObject, ~, handles)
% SEND 'D'
    GUI_Variables = handles.GUI_Variables;
    bt = GUI_Variables.BT;

    fwrite(bt,'D'); % Sends the character corresponding to ASCII value
                    % 68 to Arduino Which the Arduino understands to
                    % send parameters back
                    % Gets the Current Arduino Torque Setpoint if(strcmp(get(handles.Start_Trial,'Enable'), 'on'))
    
    GUI_Variables = Receive_Data_Message(GUI_Variables,handles);
    handles.GUI_Variables = GUI_Variables;
    guidata(hObject, handles);


function L_Setpoint_Edit_Callback(~, ~, ~)
% hObject    handle to L_Setpoint_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of L_Setpoint_Edit as text
%        str2double(get(hObject,'String')) returns contents of L_Setpoint_Edit as a double



% --- Executes during object creation, after setting all properties.
function L_Setpoint_Edit_CreateFcn(hObject, ~, ~)
% hObject    handle to L_Setpoint_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end


% --- Executes on button press in L_Set_Setpoint.
function L_Set_Setpoint_Callback(hObject, ~, handles)
% hObject    handle to L_Set_Setpoint (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% SEND 'F'
    GUI_Variables = handles.GUI_Variables;
    bt = GUI_Variables.BT;

    if(bt.Status=="open")
        fwrite(bt,char(70));
    end

    NewSetpoint = str2double(get(handles.L_Setpoint_Edit,'String')); % Gets the Value entered into the edit Box in the G
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

    pause(0.3);
    L_Get_Setpoint_Callback(hObject,0,handles);
    handles.GUI_Variables = GUI_Variables;
    guidata(hObject, handles);
    

% --- Executes on button press in Get_Smoothing.
function [n1,n2,n3]=Get_Smoothing_Callback(hObject, ~, handles)
% SEND '(' to get the smoothing
    GUI_Variables = handles.GUI_Variables;
    bt = GUI_Variables.BT;
    n1=0;
    n2=0;
    n3=0;
    if (bt.Status=="open")
        try
            fwrite(bt,'(');
            if(strcmp(get(handles.Start_Trial,'Enable'), 'on') )
                
                GUI_Variables = Receive_Data_Message(GUI_Variables,handles);

            end
        catch
            %     else
            disp("Impossible to get shaping parameters from bt");
            set(handles.N1_Text,'String',"NaN");
            set(handles.N2_Text,'String',"NaN");
            set(handles.N3_Text,'String',"NaN");
        end
    end
    handles.GUI_Variables = GUI_Variables;
    guidata(hObject, handles);

% --- Executes on button press in Set_Smoothing.
function Set_Smoothing_Callback(hObject, ~, handles)
% hObject    handle to Set_Smoothing (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    GUI_Variables = handles.GUI_Variables;
    bt = GUI_Variables.BT;

    if (bt.Status=="open")
        try
            fwrite(bt,')');
            N1 = str2double(get(handles.N1_Edit,'String')); % Gets the Value entered into the edit Box in the G
            N2 = str2double(get(handles.N2_Edit,'String'));
            N3 = str2double(get(handles.N3_Edit,'String'));
            disp('Smoothing');
            disp(N1)
            disp(N2)
            disp(N3)
            fwrite(bt,N1,'double'); % Sends the new Torque Value to Arduino
            fwrite(bt,N2,'double');
            fwrite(bt,N3,'double');
            pause(0.3);
            Get_Smoothing_Callback(hObject,0,handles);
        catch
            disp("Impossible to set shaping parameters for BTRL");
        end
    end
    

function N1_Edit_Callback(~, ~, ~)
% hObject    handle to N1_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of N1_Edit as text
%        str2double(get(hObject,'String')) returns contents of N1_Edit as a double


% --- Executes during object creation, after setting all properties.
function N1_Edit_CreateFcn(hObject, ~, ~)
% hObject    handle to N1_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end



function N2_Edit_Callback(~, ~, ~)
% hObject    handle to N2_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of N2_Edit as text
%        str2double(get(hObject,'String')) returns contents of N2_Edit as a double


% --- Executes during object creation, after setting all properties.
function N2_Edit_CreateFcn(hObject, ~, ~)
% hObject    handle to N2_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end



function N3_Edit_Callback(~, ~, ~)
% hObject    handle to N3_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of N3_Edit as text
%        str2double(get(hObject,'String')) returns contents of N3_Edit as a double


% --- Executes during object creation, after setting all properties.
function N3_Edit_CreateFcn(hObject, ~, ~)
% hObject    handle to N3_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end

% --- Executes on button press in R_N3_Adj.
function R_N3_Adj_Callback(hObject, ~, handles)
% hObject    handle to R_N3_Adj (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    GUI_Variables = handles.GUI_Variables;
    bt = GUI_Variables.BT;
    if (bt.Status=="open")
        try
            fwrite(bt,'o'); % char(111)
            count_speed=GUI_Variables.RLCount;
            GUI_Variables.R_COUNT_SPEED=[GUI_Variables.R_COUNT_SPEED;[count_speed,2]];
        catch
        end
    end
    handles.GUI_Variables = GUI_Variables;
    guidata(hObject, handles);

% --- Executes on button press in L_N3_Adj.
function L_N3_Adj_Callback(hObject, ~, handles)
% hObject    handle to L_N3_Adj (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    GUI_Variables = handles.GUI_Variables;
    bt = GUI_Variables.BT;
    if (bt.Status=="open")
        try
            fwrite(bt,'O'); % char(79)
            count_speed=GUI_Variables.RLCount;
            GUI_Variables.L_COUNT_SPEED=[GUI_Variables.L_COUNT_SPEED;[count_speed,2]];
        catch
        end
    end
    handles.GUI_Variables = GUI_Variables;
    guidata(hObject, handles);

% --- Executes on button press in L_Bs_Frq.
function L_Bs_Frq_Callback(~, ~, handles)
% hObject    handle to L_Bs_Frq (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    GUI_Variables = handles.GUI_Variables;
    bt = GUI_Variables.BT;

    if (bt.Status=="open")
        try
            fwrite(bt,'P'); % char(80)
        catch
        end
    end


function R_Send_KF_Edit_Callback(~, ~, ~)
% hObject    handle to R_Send_KF_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of R_Send_KF_Edit as text
%        str2double(get(hObject,'String')) returns contents of R_Send_KF_Edit as a double


% --- Executes during object creation, after setting all properties.
function R_Send_KF_Edit_CreateFcn(hObject, ~, ~)
% hObject    handle to R_Send_KF_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end


% --- Executes on button press in R_Send_KF.
function R_Send_KF_Callback(hObject, ~, handles)
% hObject    handle to R_Send_KF (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    new_KF = str2double(get(handles.R_Send_KF_Edit,'String')); % Gets the Value entered into the edit Box in the G

    GUI_Variables = handles.GUI_Variables;
    state=GUI_Variables.state;
    disp(state);

    bt = GUI_Variables.BT;

    if (bt.Status=="open")
        try
            fwrite(bt,'-'); % send the character "-"
            fwrite(bt,new_KF,'double');
            disp("Send new Right KF ");
            disp(new_KF);

        catch
            disp("Impossible to write on bt the new KF");
        end
    end
    R_Check_KF_Callback(hObject, 0, handles);
    handles.GUI_Variables = GUI_Variables;
    guidata(hObject, handles);

% --- Executes on button press in R_Check_KF.
function rkf=R_Check_KF_Callback(hObject, ~, handles)
    GUI_Variables = handles.GUI_Variables;
    bt = GUI_Variables.BT;
    rkf=0;
    if (bt.Status=="open")
        try

            fwrite(bt,'~'); % send the character "~"
            if (strcmp(get(handles.Start_Trial,'Enable'), 'on'))

                GUI_Variables = Receive_Data_Message(GUI_Variables,handles);
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
    handles.GUI_Variables = GUI_Variables;
    guidata(hObject, handles);

% --- Executes on button press in L_Check_FSR_Th.
function lfsr=L_Check_FSR_Th_Callback(hObject, ~, handles)
    GUI_Variables = handles.GUI_Variables;
    bt = GUI_Variables.BT;
    lfsr=0;
    if (bt.Status=="open")
        try
            fwrite(bt,char('Q')); % send the character "Q"
            GUI_Variables = Receive_Data_Message(GUI_Variables,handles);
        catch
            disp("Impossible to know L FSR TH");
            set(handles.R_Check_FSR_Text,'String',"NaN");
        end
    end
    if (bt.Status=="closed")
        disp("Impossible to know FSR THs");
        set(handles.L_Check_FSR_Text,'String',"NaN");
        set(handles.R_Check_FSR_Text,'String',"NaN");
    end
    handles.GUI_Variables = GUI_Variables;
    guidata(hObject, handles);


function L_Send_FSR_Edit_Callback(~, ~, ~)
% hObject    handle to L_Send_FSR_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of L_Send_FSR_Edit as text
%        str2double(get(hObject,'String')) returns contents of L_Send_FSR_Edit as a double


% --- Executes during object creation, after setting all properties.
function L_Send_FSR_Edit_CreateFcn(hObject, ~, ~)
% hObject    handle to L_Send_FSR_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end


% --- Executes on button press in L_Send_FSR_Th.
function L_Send_FSR_Th_Callback(hObject, ~, handles)
% hObject    handle to L_Send_FSR_Th (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    GUI_Variables = handles.GUI_Variables;
    bt = GUI_Variables.BT;


    if (bt.Status=="open")
        try
            fwrite(bt,'R'); % char 35 -> #, 36 -> $, 74-> J
            LFSRTH = str2double(get(handles.L_Send_FSR_Edit,'String')); % Gets the Value entered into the edit Box in the G
            fwrite(bt,LFSRTH,'double'); %Sends the new Torque Value to Arduino
            L_Check_FSR_Th_Callback(hObject, 0, handles);
        catch
            disp("Impossible to set FSR th parameters for Left");
        end
    end

% --- Executes on button press in R_Check_FSR_Th.
function rfsr=R_Check_FSR_Th_Callback(~, ~, handles)
    GUI_Variables = handles.GUI_Variables;
    bt = GUI_Variables.BT;
    rfsr=0;
    if (bt.Status=="open")
        try
            fwrite(bt,char('q')); % send the character "Q"
            [message,data] = get_message(bt);
            if message == 'q'
                Curr_TH_R = data(1);
                set(handles.R_Check_FSR_Text,'String',Curr_TH_R);
                disp("Right Current FSR th ");
                disp(Curr_TH_R);
                rfsr=Curr_TH_R;
            end
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


function R_Send_FSR_Edit_Callback(hObject, ~, ~)
% hObject    handle to R_Send_FSR_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of R_Send_FSR_Edit as text
%        str2double(get(hObject,'String')) returns contents of R_Send_FSR_Edit as a double
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end



% --- Executes during object creation, after setting all properties.
function R_Send_FSR_Edit_CreateFcn(hObject, ~, ~)
% hObject    handle to R_Send_FSR_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end


% --- Executes on button press in R_Send_FSR_Th.
function R_Send_FSR_Th_Callback(hObject, ~, handles)
% hObject    handle to R_Send_FSR_Th (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    GUI_Variables = handles.GUI_Variables;
    bt = GUI_Variables.BT;


    if (bt.Status=="open")
        try
            fwrite(bt,'r'); % char 35 -> #, 36 -> $, 74-> J
            RFSRTH = str2double(get(handles.R_Send_FSR_Edit,'String')); % Gets the Value entered into the edit Box in the G
            fwrite(bt,RFSRTH,'double'); % Sends the new Torque Value to Arduino
            R_Check_FSR_Th_Callback(hObject, 0, handles);
        catch
            disp("Impossible to set FSR th parameters for Right");
        end
    end

% --- Executes on button press in Enable_Arduino_Trig.
function Enable_Arduino_Trig_Callback(~, ~, ~)
% hObject    handle to Enable_Arduino_Trig (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of Enable_Arduino_Trig

function L_Ki_Edit_Callback(~, ~, ~)
% hObject    handle to L_Ki_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of L_Ki_Edit as text
%        str2double(get(hObject,'String')) returns contents of L_Ki_Edit as a double


% --- Executes during object creation, after setting all properties.
function L_Ki_Edit_CreateFcn(hObject, ~, ~)
% hObject    handle to L_Ki_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end


% --- Executes on button press in L_Set_Perc.
function L_Set_Perc_Callback(~, ~, handles)
% hObject    handle to L_Set_Perc (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    GUI_Variables = handles.GUI_Variables;
    bt = GUI_Variables.BT;

    if (bt.Status=="open")
        try
            fwrite(bt,'S');
            L_PERC = str2double(get(handles.L_Set_Perc_Edit,'String')); % Gets the Value entered into the edit Box in the G
            fwrite(bt,L_PERC,'double'); % Sends the new Torque Value to Arduino
        catch
            disp("Impossible to set Left Perc parameter for Left");
        end
    end


function L_Set_Perc_Edit_Callback(~, ~, ~)
% hObject    handle to L_Set_Perc_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of L_Set_Perc_Edit as text
%        str2double(get(hObject,'String')) returns contents of L_Set_Perc_Edit as a double


% --- Executes during object creation, after setting all properties.
function L_Set_Perc_Edit_CreateFcn(hObject, ~, ~)
% hObject    handle to L_Set_Perc_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end

% --- Executes on button press in R_Set_Perc.
function R_Set_Perc_Callback(~, ~, handles)
% hObject    handle to R_Set_Perc (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    GUI_Variables = handles.GUI_Variables;
    bt = GUI_Variables.BT;

    if (bt.Status=="open")
        try
            fwrite(bt,'s');
            R_PERC = str2double(get(handles.R_Set_Perc_Edit,'String')); % Gets the Value entered into the edit Box in the G
            fwrite(bt,R_PERC,'double'); % Sends the new Torque Value to Arduino
        catch
            disp("Impossible to set Left Perc parameter for Right");
        end
    end


function R_Set_Perc_Edit_Callback(~, ~, ~)
% hObject    handle to R_Set_Perc_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of R_Set_Perc_Edit as text
%        str2double(get(hObject,'String')) returns contents of R_Set_Perc_Edit as a double


% --- Executes during object creation, after setting all properties.
function R_Set_Perc_Edit_CreateFcn(hObject, ~, ~)
% hObject    handle to R_Set_Perc_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end


% --- Executes on button press in L_Stop_N3.
function L_Stop_N3_Callback(hObject, ~, handles)
% hObject    handle to L_Stop_N3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    GUI_Variables = handles.GUI_Variables;
    bt = GUI_Variables.BT;
    if (bt.Status=="open")
        try
            fwrite(bt,'T');
            count_speed=GUI_Variables.RLCount;
            GUI_Variables.L_COUNT_SPEED=[GUI_Variables.L_COUNT_SPEED;[count_speed,1]];

        catch
        end
    end
    handles.GUI_Variables = GUI_Variables;
    guidata(hObject, handles);

% --- Executes on button press in L_Stop_Trq.
function L_Stop_Trq_Callback(~, ~, handles)
% hObject    handle to L_Stop_Trq (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    GUI_Variables = handles.GUI_Variables;
    bt = GUI_Variables.BT;

    if (bt.Status=="open")
        try
            fwrite(bt,'I');
        catch
        end
    end

% --- Executes on button press in R_Stop_N3.
function R_Stop_N3_Callback(hObject, ~, handles)
% hObject    handle to R_Stop_N3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    GUI_Variables = handles.GUI_Variables;
    bt = GUI_Variables.BT;
    if (bt.Status=="open")
        try
            fwrite(bt,'t');
            count_speed=GUI_Variables.RLCount;
            GUI_Variables.R_COUNT_SPEED=[GUI_Variables.R_COUNT_SPEED;[count_speed,0]];

        catch
        end
    end
    handles.GUI_Variables = GUI_Variables;
    guidata(hObject, handles);

% --- Executes on button press in R_Stop_Trq.
function R_Stop_Trq_Callback(~, ~, handles)
% hObject    handle to R_Stop_Trq (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    GUI_Variables = handles.GUI_Variables;
    bt = GUI_Variables.BT;

    if (bt.Status=="open")
        try
            fwrite(bt,'i');
        catch
        end
    end

% --- Executes on button press in Save_EXP_Prm.
function Save_EXP_Prm_Callback(~, ~, handles)
% hObject    handle to Save_EXP_Prm (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    GUI_Variables = handles.GUI_Variables;
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
    GUI_Variables = handles.GUI_Variables;
    bt=GUI_Variables.BT;
    disp("Before Starting we check the connections");
    set(handles.statusText,'String','Checking Connections');
    pause(0.01)

    disp("Check BT")
    Check_Bluetooth_Callback(hObject, eventdata, handles);
    set(handles.statusText,'String','Connections Checked');
    pause(0.01)

    valBT = bt.Status == 'open';

    if valBT==0
        set(handles.statusText,'String','Connect BT before');
        return
    end

    disp('Load from File ');
    strfile=input('Which file do you want to upload: ','s');

    if not(exist(strfile, 'file'))
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
function L_InverseSign_RadioButton_Callback(hObject, ~, handless)
% hObject    handle to L_InverseSign_RadioButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of L_InverseSign_RadioButton
% Hint: get(hObject,'Value') returns toggle state of L_InverseSign_RadioButton

    GUI_Variables = handles.GUI_Variables;
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
function R_InverseSign_RadioButton_Callback(hObject, ~, handles)
% hObject    handle to R_InverseSign_RadioButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of R_InverseSign_RadioButton


    GUI_Variables = handles.GUI_Variables;
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
function R_Check_Gain_Callback(hObject, ~, handles)
% hObject    handle to R_Check_Gain (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    GUI_Variables = handles.GUI_Variables;
    bt = GUI_Variables.BT;

    if(bt.Status=="open")
        fwrite(bt,']');
    end

    if(strcmp(get(handles.Start_Trial,'Enable'), 'on'))
        
        GUI_Variables = Receive_Data_Message(GUI_Variables,handles);

    end
    handles.GUI_Variables = GUI_Variables;
    guidata(hObject, handles);

% --- Executes on button press in R_Set_Gain.
function R_Set_Gain_Callback(~, ~, handles)
% hObject    handle to R_Set_Gain (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    GUI_Variables = handles.GUI_Variables;
    bt = GUI_Variables.BT;

    if(bt.Status=="open")
        fwrite(bt,'[');
    end

    try
        R_New_Gain = str2double(get(handles.R_Set_Gain_Edit,'String')); % Gets the Value entered into the edit Box in the G
        fwrite(bt,R_New_Gain,'double');
        disp('Send to arduino Right Prop Gain');
        disp(R_New_Gain);
    catch
    end



function R_Set_Gain_Edit_Callback(~, ~, ~)
% hObject    handle to R_Set_Gain_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of R_Set_Gain_Edit as text
%        str2double(get(hObject,'String')) returns contents of R_Set_Gain_Edit as a double


% --- Executes during object creation, after setting all properties.
function R_Set_Gain_Edit_CreateFcn(hObject, ~, ~)
% hObject    handle to R_Set_Gain_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end


% --- Executes on button press in L_Check_Gain.
function L_Check_Gain_Callback(hObject, ~, handles)
% hObject    handle to L_Check_Gain (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

    GUI_Variables = handles.GUI_Variables;
    bt = GUI_Variables.BT;

    try
        if(bt.Status=="open")
            fwrite(bt,'}');
        end

        if(strcmp(get(handles.Start_Trial,'Enable'), 'on'))

            GUI_Variables = Receive_Data_Message(GUI_Variables,handles);

        end
    catch
    end
    handles.GUI_Variables = GUI_Variables;
    guidata(hObject, handles);


% --- Executes on button press in L_Set_Gain.
function L_Set_Gain_Callback(~, ~, handles)
% hObject    handle to L_Set_Gain (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    GUI_Variables = handles.GUI_Variables;
    bt = GUI_Variables.BT;

    try
        if(bt.Status=="open")
            fwrite(bt,'{');
        end
        L_New_Gain = str2double(get(handles.L_Set_Gain_Edit,'String')); % Gets the Value entered into the edit Box in the G
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





function L_Set_Gain_Edit_Callback(~, ~, ~)
% hObject    handle to L_Set_Gain_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of L_Set_Gain_Edit as text
%        str2double(get(hObject,'String')) returns contents of L_Set_Gain_Edit as a double


% --- Executes during object creation, after setting all properties.
function L_Set_Gain_Edit_CreateFcn(hObject, ~, ~)
% hObject    handle to L_Set_Gain_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end


% --- Executes on button press in Activate_Balance.
function Activate_Balance_Callback(~, ~, handles)
% hObject    handle to Activate_Balance (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

    GUI_Variables = handles.GUI_Variables;
    bt = GUI_Variables.BT;

    % BT_auto_reconnect_radiobutton
    if (bt.Status=="open")
        Balance=get(handles.Balance_Text,'String');

        if strcmp(Balance,'On')
            %deactivate prop control
            fwrite(bt,'=');
            disp('Deactivate Balance Ctrl');
            set(handles.Balance_Text,'String','Off')
        else
            %activate prop control

            fwrite(bt,'+');
            disp('Activate Balance Ctrl');
            set(handles.Balance_Text,'String','On')
        end


    end




% --- Executes on button press in radiobutton12.
function radiobutton12_Callback(~, ~, ~)
% hObject    handle to radiobutton12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton12


% --- Executes on button press in radiobutton11.
function radiobutton11_Callback(~, ~, ~)
% hObject    handle to radiobutton11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton11


% --- Executes during object creation, after setting all properties.
function R_Torque_CreateFcn(~, ~, ~)
% hObject    handle to R_Torque (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes on button press in Activate_Balance.
function radiobutton13_Callback(~, ~, ~)
% hObject    handle to Activate_Balance (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of Activate_Balance


% --- Executes on button press in L_Auto_KF.
function L_Auto_KF_Callback(hObject, ~, handles)
% hObject    handle to L_Auto_KF (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of L_Auto_KF
    GUI_Variables = handles.GUI_Variables;
    bt = GUI_Variables.BT;

    if (bt.Status=="open")

        LKF=get(hObject,'Value');
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
   

% --- Executes on button press in Fast_0_Trq.
function Fast_0_Trq_Callback(~, ~, handles)
% hObject    handle to Fast_0_Trq (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% SEND 'F'
    GUI_Variables = handles.GUI_Variables;
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
function Slow_0_Trq_Callback(~, ~, handles)
% hObject    handle to Slow_0_Trq (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

GUI_Variables = handles.GUI_Variables;
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
function Take_Baseline_Callback(~, ~, handles)
% hObject    handle to Take_Baseline (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

GUI_Variables = handles.GUI_Variables;
bt = GUI_Variables.BT;

try
    if(bt.Status=="open")
        fwrite(bt,'b');
        set(handles.statusText,'String','Taking baseline for proportional...');
    end



catch
end


% --- Executes on button press in Check_Baseline.
function Check_Baseline_Callback(hObject, ~, handles)
% hObject    handle to Check_Baseline (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
GUI_Variables = handles.GUI_Variables;
bt = GUI_Variables.BT;

try
    if(bt.Status=="open")
        fwrite(bt,'B');
    end

    val=get(handles.Activate_Balance,'Value');


    disp('Check Baseline');

    GUI_Variables = Receive_Data_Message(GUI_Variables,handles);
catch
end
handles.GUI_Variables = GUI_Variables;
guidata(hObject, handles);



function L_Setpoint_Dorsi_Edit_Callback(~, ~, ~)
% hObject    handle to L_Setpoint_Dorsi_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of L_Setpoint_Dorsi_Edit as text
%        str2double(get(hObject,'String')) returns contents of L_Setpoint_Dorsi_Edit as a double


% --- Executes during object creation, after setting all properties.
function L_Setpoint_Dorsi_Edit_CreateFcn(hObject, ~, ~)
% hObject    handle to L_Setpoint_Dorsi_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end



function R_Setpoint_Dorsi_Edit_Callback(~, ~, ~)
% hObject    handle to R_Setpoint_Dorsi_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of R_Setpoint_Dorsi_Edit as text
%        str2double(get(hObject,'String')) returns contents of R_Setpoint_Dorsi_Edit as a double


% --- Executes during object creation, after setting all properties.
function R_Setpoint_Dorsi_Edit_CreateFcn(hObject, ~, ~)
% hObject    handle to R_Setpoint_Dorsi_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end



function L_Zero_Modif_Edit_Callback(~, ~, ~)
% hObject    handle to L_Zero_Modif_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of L_Zero_Modif_Edit as text
%        str2double(get(hObject,'String')) returns contents of L_Zero_Modif_Edit as a double


% --- Executes during object creation, after setting all properties.
function L_Zero_Modif_Edit_CreateFcn(hObject, ~, ~)
% hObject    handle to L_Zero_Modif_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end


% --- Executes on button press in L_Set_Zero_Modif.
function L_Set_Zero_Modif_Callback(~, ~, handles)
% hObject    handle to L_Set_Zero_Modif (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    GUI_Variables = handles.GUI_Variables;
    bt = GUI_Variables.BT;

    try
        if(bt.Status=="open")
            fwrite(bt,'W');
        end

        L_Zero_Modif = str2double(get(handles.L_Zero_Modif_Edit,'String')); % Gets the Value entered into the edit Box in the G
        fwrite(bt,L_Zero_Modif,'double');
        disp(L_Zero_Modif);
    catch
    end


function R_Zero_Modif_Edit_Callback(~, ~, ~)
% hObject    handle to R_Zero_Modif_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of R_Zero_Modif_Edit as text
%        str2double(get(hObject,'String')) returns contents of R_Zero_Modif_Edit as a double


% --- Executes during object creation, after setting all properties.
function R_Zero_Modif_Edit_CreateFcn(hObject, ~, ~)
% hObject    handle to R_Zero_Modif_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end


% --- Executes on button press in R_Set_Zero_Modif.
function R_Set_Zero_Modif_Callback(~, ~, handles)
% hObject    handle to R_Set_Zero_Modif (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    GUI_Variables = handles.GUI_Variables;
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
function Balance_Baseline_Callback(~, ~, ~)
% hObject    handle to Balance_Baseline (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes on button press in BT_auto_reconnect_radiobutton.
function BT_auto_reconnect_radiobutton_Callback(hObject, ~, handles)
% hObject    handle to BT_auto_reconnect_radiobutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of BT_auto_reconnect_radiobutton

    GUI_Variables = handles.GUI_Variables;
    bt = GUI_Variables.BT;

    if (bt.Status=="open")
        BT_auto=get(hObject,'Value');
        try
            if BT_auto
                %activate prop control
                fwrite(bt,'|');
                disp('Activate BT auto reconnect');
            else
                %deactivate prop control
                fwrite(bt,'@');
                disp('Deactivate BT auto reconnect');
            end
        catch
        end
    end



% --- Executes on button press in Steady_Balance_Base.
function Steady_Balance_Base_Callback(~, ~, handles)
% hObject    handle to Steady_Balance_Base (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


    GUI_Variables = handles.GUI_Variables;
    bt = GUI_Variables.BT;

    try
        if(bt.Status=="open")
            fwrite(bt,'J');
        end

        disp('Steady Balance Baseline');
    catch
    end



% --- Executes on button press in Dynamic_Balance_Base.
function Dynamic_Balance_Base_Callback(~, ~, handles)
% hObject    handle to Dynamic_Balance_Base (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    GUI_Variables = handles.GUI_Variables;
    bt = GUI_Variables.BT;

    try
        if(bt.Status=="open")
            fwrite(bt,'&');
        end

        disp('Balance Baseline');
    catch
    end



% --- Executes on button press in Set_Steady_Val.
function Set_Steady_Val_Callback(~, ~, handles)
% hObject    handle to Set_Steady_Val (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    GUI_Variables = handles.GUI_Variables;
    bt = GUI_Variables.BT;

    if(bt.Status=="open")
        try
            fwrite(bt,'v');
            steady_val = str2double(get(handles.Steady_Edit,'String'));
            fwrite(bt,steady_val,'double');
            disp(['Steady Val ',num2str(steady_val)]);
        catch
        end
    end


% --- Executes on button press in Check_Steady_Val.
function Check_Steady_Val_Callback(hObject, ~, handles)
% hObject    handle to Check_Steady_Val (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    GUI_Variables = handles.GUI_Variables;
    bt=GUI_Variables.BT;
    if(bt.Status=="open")
        try
            fwrite(bt,'V');
            if(strcmp(get(handles.Start_Trial,'Enable'), 'on') )
                GUI_Variables = Receive_Data_Message(GUI_Variables, handles);
            end
        catch
        end
    end
    handles.GUI_Variables = GUI_Variables;
    guidata(hObject, handles);

% --- Executes on button press in Set_Dyn_Val.
function Set_Dyn_Val_Callback(~, ~, handles)
% hObject    handle to Set_Dyn_Val (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    GUI_Variables = handles.GUI_Variables;
    bt = GUI_Variables.BT;

    if(bt.Status=="open")
        try
            fwrite(bt,'a');
            dyn_val = str2double(get(handles.Dyn_Edit,'String'));
            fwrite(bt,dyn_val,'double');
            disp(['Dynamic Val ',num2str(dyn_val)]);
        catch
        end
    end

% --- Executes on button press in Check_Dyn_Val.
function Check_Dyn_Val_Callback(hObject, ~, handles)
% hObject    handle to Check_Dyn_Val (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    GUI_Variables = handles.GUI_Variables;
    bt = GUI_Variables.BT;

    if(bt.Status=="open")
        try
            fwrite(bt,'A');

            if(strcmp(get(handles.Start_Trial,'Enable'), 'on') )
                if(strcmp(get(handles.Start_Trial,'Enable'), 'on') )
                    GUI_Variables = Receive_Data_Message(GUI_Variables, handles);
                end
            end

        catch
        end
    end
    handles.GUI_Variables = GUI_Variables;
    guidata(hObject, handles);


function Steady_Edit_Callback(~, ~, ~)
% hObject    handle to Steady_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Steady_Edit as text
%        str2double(get(hObject,'String')) returns contents of Steady_Edit as a double


% --- Executes during object creation, after setting all properties.
function Steady_Edit_CreateFcn(hObject, ~, ~)
% hObject    handle to Steady_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end



function Dyn_Edit_Callback(~, ~, ~)
% hObject    handle to Dyn_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Dyn_Edit as text
%        str2double(get(hObject,'String')) returns contents of Dyn_Edit as a double


% --- Executes during object creation, after setting all properties.
function Dyn_Edit_CreateFcn(hObject, ~, ~)
% hObject    handle to Dyn_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end

% --- Executes on button press Flush Bluetooth.
function Flush_Biobluetooth_Callback(~, ~, handles)
% hObject    handle to Flush_Bluetooth (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    GUI_Variables = handles.GUI_Variables;
    bt = GUI_Variables.BT;
    if (bt.Status=="open")
        fwrite(bt,'Y');
        disp('Deactivate BT auto reconnect');
    end



% --- Executes on button press in BT_Auto_Reconnection.
function BT_Auto_Reconnection_Callback(~, ~, handles)
% hObject    handle to BT_Auto_Reconnection (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

    GUI_Variables = handles.GUI_Variables;
    bt = GUI_Variables.BT;

    BT_auto=get(handles.BT_Text,'String');

    if strcmp(BT_auto,'On')
        %deactivate prop control
        if (bt.Status=="open")
            fwrite(bt,'@');
            disp('Deactivate BT auto reconnect');
        end
        set(handles.BT_Text,'String','Off')
    else
        %activate prop control
        if (bt.Status=="open")
            fwrite(bt,'|');
            disp('Activate BT auto reconnect');
        end
        set(handles.BT_Text,'String','On')
    end



% --- Executes on button press in radiobutton23.
function radiobutton23_Callback(~, ~, ~)
% hObject    handle to radiobutton23 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton23



% --- Executes on button press in Activate_BioFeedback.
function Activate_BioFeedback_Callback(~, ~, handles)
% hObject    handle to Activate_BioFeedback (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    GUI_Variables = handles.GUI_Variables;
    bt = GUI_Variables.BT;

    % BT_auto_reconnect_radiobutton
    if (bt.Status=="open")
        BIO=get(handles.Activate_BioFeedback_Text,'String');

        if strcmp(BIO,'On')
            %deactivate
            fwrite(bt,'y');
            disp('Deactivate Audio Bio Feedback');
            set(handles.Activate_BioFeedback_Text,'String','Off')
        else
            %activate
            fwrite(bt,'/');
            disp('Activate Audio Bio Feedback');
            set(handles.Activate_BioFeedback_Text,'String','On')
        end


    end

% --- Executes on button press in Set_Bias.
function Set_Bias_Callback(~, ~, handles)
% hObject    handle to Set_Bias (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    GUI_Variables = handles.GUI_Variables;
    bt = GUI_Variables.BT;

    if (bt.Status=="open")
        try
            bias=str2double(get(handles.Set_Bias_Edit,'String'));
            if not(isempty(bias))
                fwrite(bt,'*');
                fwrite(bt,bias,'double');
                disp(['target is ',num2str(bias), ' times baseline']);
            end

        catch
        end

    end


% --- Executes on button press in Set_Target.
function Set_Target_Callback(~, ~, handles)
% hObject    handle to Set_Target (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    GUI_Variables = handles.GUI_Variables;
    bt = GUI_Variables.BT;

    if (bt.Status=="open")
        try
            target=str2double(get(handles.Set_Target_Edit,'String'));
            fwrite(bt,'u');
            if not(isempty(target))

                fwrite(bt,target,'double');
                disp(['BioFeedback Target ',num2str(target)]);
            end

        catch
        end
    end


function Set_Bias_Edit_Callback(~, ~, ~)
% hObject    handle to Set_Bias_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Set_Bias_Edit as text
%        str2double(get(hObject,'String')) returns contents of Set_Bias_Edit as a double


% --- Executes during object creation, after setting all properties.
function Set_Bias_Edit_CreateFcn(hObject, ~, ~)
% hObject    handle to Set_Bias_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end



function Set_Target_Edit_Callback(~, ~, ~)
% hObject    handle to Set_Target_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Set_Target_Edit as text
%        str2double(get(hObject,'String')) returns contents of Set_Target_Edit as a double


% --- Executes during object creation, after setting all properties.
function Set_Target_Edit_CreateFcn(hObject, ~, ~)
% hObject    handle to Set_Target_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end

% --- Executes on selection change in IP_list.
function IP_list_Callback(hObject, ~, handles)
% hObject    handle to IP_list (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    GUI_Variables = handles.GUI_Variables;
    selectMode = get(handles.IP_list,'Value');
    if selectMode == 1
        GUI_Variables.IP = '10.18.48.77';
    elseif selectMode == 2
        GUI_Variables.IP = '10.18.48.128';
    elseif selectMode == 3
        GUI_Variables.IP = '10.18.48.160';
    elseif selectMode == 4
        GUI_Variables.IP = '10.18.48.166';
    end
    handles.GUI_Variables = GUI_Variables;
    guidata(hObject, handles);


% --- Executes on button press in Start_TCP.
function Open_TCP_Callback(hObject, ~, handles)
% hObject    handle to Start_TCP (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

    GUI_Variables = handles.GUI_Variables;

    if strcmp(GUI_Variables.IP,'0.0.0.0')
        set(handles.statusText,'String',"Select host IP address from drop-down");
    else
        try
            fprintf('Attempting to make a TCP/IP connection to the host');
            fprintf('\n');
            set(handles.statusText,'String',"Attempting to make a TCP/IP connection!");
            pause(.01);
            t = tcpip(GUI_Variables.IP,3000,'NetworkRole','client');   %Creates TCP/IP connection client object
            GUI_Variables.t = t;
            fopen(t);       %Open TCP communication port
        catch
            set(handles.TCP_Status,'Color',[1 0 0]);
            set(handles.statusText,'String',"TCP/IP connection failed. Check host IP and try again.");
        end
    end

    if exist('t','var')
        if(t.Status == "open")
            set(handles.TCP_Status,'Color',[0 1 0]);
            fprintf('Made a TCP connection to the host!');
            fprintf('\n');
            set(handles.statusText,'String',"Made a TCP connection to the host!");
        elseif(t.Status == "closed")
            set(handles.TCP_Status,'Color',[1 0 0]);
            set(handles.statusText,'String',"Could not establish TCP connection to host within time limit. Check host IP and try again.");
        end
    end
    handles.GUI_Variables = GUI_Variables;
    guidata(hObject, handles);


% --- Executes on button press in Close_TCP.
function Close_TCP_Callback(~, ~, handles)
% hObject    handle to Close_TCP (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

    GUI_Variables = handles.GUI_Variables;
    try
        t = GUI_Variables.t;
    catch
    end

    if exist('t','var')
        if (t.Status == "open")
            fclose(t);      %Close TCP connection
            set(handles.TCP_Status,'Color',[1 1 1]);
            fprintf('Closing TCP connection with the host!');
            fprintf('\n');
            set(handles.statusText,'String',"Closed TCP connection.");
        elseif (t.Status == "closed")
            fprintf('No TCP connection to close. \n');
            set(handles.statusText,'String',"No TCP connection to close.");
        end
    end

% --- Executes on button press in Start_Optimization.
function Start_Optimization_Callback(~, ~, handles)
% hObject    handle to Start_Optimization (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

    GUI_Variables = handles.GUI_Variables;
    try
        t = GUI_Variables.t;
    catch
        fprintf('No TCP connection established, cannot start optimization');
        fprintf('\n');
        set(handles.statusText,'String',"No TCP connection established, cannot start optimization.")
    end

    try
        bt = GUI_Variables.BT;
    catch
        fprintf('No BT connection established, cannot send start byte');
        fprintf('\n')
        set(handles.statusText,'String',"No BT connection established, cannot start optimization.")
    end

    if exist('t','var')
        if t.Status == "open" && bt.Status == "open"
            fwrite(t,"start")
            fwrite(bt,'%')
            set(handles.statusText,'String',"Initializing optimization...")
        elseif (t.Status == "closed")
            set(handles.statusText,'String',"TCP port is not open! Re-open connection.")
        elseif bt.Status == "closed"
            set(handles.statusText,'String',"No BT connection established, no optimization values to be sent to exo.")
            pause(0.1)
            fwrite(t,"start")
            set(handles.statusText,'String',"Initializing optimization (no BT)...")
        end
    end


% --- Executes on button press in Stop_Optimization.
function Stop_Optimization_Callback(~, ~, handles)
% hObject    handle to Stop_Optimization (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    GUI_Variables = handles.GUI_Variables;
    try
        t = GUI_Variables.t;
        bt = GUI_Variables.BT;
    catch
        fprintf('No TCP connection established, cannot start optimization.');
        fprintf('\n');
        set(handles.statusText,'String',"No TCP connection established, cannot start optimization.")
    end

    if exist('t','var')
        if t.Status == "open" && bt.Status == "open"
            fwrite(t,"end")
            fwrite(bt,'h')
            set(handles.statusText,'String',"Stopping optimization...")
        elseif (t.Status == "closed")
            set(handles.statusText,'String',"TCP port is not open! Re-open connection.")
        elseif bt.Status == "closed"
            fwrite(t,"end")
            set(handles.statusText,'String',"Stopping optimization...")
        end
    end



% --- Executes during object creation, after setting all properties.
function IP_list_CreateFcn(hObject, ~, ~)
% hObject    handle to IP_list (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end


% --- Executes on button press in BioFeedback_Baseline.
function BioFeedback_Baseline_Callback(~, ~, handles)
% hObject    handle to BioFeedback_Baseline (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    GUI_Variables = handles.GUI_Variables;
    bt = GUI_Variables.BT;

    if (bt.Status=="open")
        try

            fwrite(bt,':');

            set(handles.statusText,'String','Taking BioFeedback Baseline Finished');
        catch
        end
    end

% --- Executes during object creation, after setting all properties.
function TCP_Status_CreateFcn(~, ~, ~)
% hObject    handle to TCP_Status (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate TCP_Status


% --- Executes during object creation, after setting all properties.
function Open_TCP_CreateFcn(~, ~, ~)
% hObject    handle to Open_TCP (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes during object creation, after setting all properties.
function Close_TCP_CreateFcn(~, ~, ~)
% hObject    handle to Close_TCP (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes during object creation, after setting all properties.
function Start_Optimization_CreateFcn(~, ~, ~)
% hObject    handle to Start_Optimization (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes during object creation, after setting all properties.
function Stop_Optimization_CreateFcn(~, ~, ~)
% hObject    handle to Stop_Optimization (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called



function FSR_Distance_In_Callback(~, ~, ~)
% hObject    handle to FSR_Distance_In (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of FSR_Distance_In as text
%        str2double(get(hObject,'String')) returns contents of FSR_Distance_In as a double


% --- Executes during object creation, after setting all properties.
function FSR_Distance_In_CreateFcn(hObject, ~, ~)
% hObject    handle to FSR_Distance_In (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end



function Ankle2FSR_Distance_In_Callback(~, ~, ~)
% hObject    handle to Ankle2FSR_Distance_In (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Ankle2FSR_Distance_In as text
%        str2double(get(hObject,'String')) returns contents of Ankle2FSR_Distance_In as a double


% --- Executes during object creation, after setting all properties.
function Ankle2FSR_Distance_In_CreateFcn(hObject, ~, ~)
% hObject    handle to Ankle2FSR_Distance_In (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end



function SSID_Input_Callback(hObject, ~, handles)
% hObject    handle to SSID_Input (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

    GUI_Variables = handles.GUI_Variables;

    GUI_Variables.SSID = get(hObject,'String');
    set(handles.statusText,'String',['Subject ID entered: ',GUI_Variables.SSID]);
    handles.GUI_Variables = GUI_Variables;
    guidata(hObject, handles);



% Hints: get(hObject,'String') returns contents of SSID_Input as text
%        str2double(get(hObject,'String')) returns contents of SSID_Input as a double


% --- Executes during object creation, after setting all properties.
function SSID_Input_CreateFcn(hObject, ~, ~)
% hObject    handle to SSID_Input (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end


% --- Executes on button press in Version_Button.
function Version_Button_Callback(hObject, ~, handles)
% hObject    handle to Version_Button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    GUI_Variables = handles.GUI_Variables;
    bt = GUI_Variables.BT;

    if (bt.Status=="open")
        try
            fwrite(bt,'U');
            GUI_Variables = Receive_Data_Message(GUI_Variables, handles);
        catch E 
        end
    end
    handles.GUI_Variables = GUI_Variables;
    guidata(hObject, handles);


% --- Executes on button press in Motor_Error.
function Motor_Error_Callback(hObject, ~, handles)
% hObject    handle to Motor_Error (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of Motor_Error
    GUI_Variables = handles.GUI_Variables;
    bt = GUI_Variables.BT;

    if (bt.Status=="open")
        try
            fwrite(bt,'z');
            GUI_Variables = Receive_Data_Message(GUI_Variables, handles);
        catch E
        end
    end
    handles.GUI_Variables = GUI_Variables;
    guidata(hObject, handles);
    
    

% --- Executes on button press in Torque.
function Torque_Callback(hObject, eventdata, handles)
% hObject    handle to Torque (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
        
        set(handles.L_Torque,'Visible','on');
        set(handles.L_PID,'Visible','off');
        set(handles.L_Adj,'Visible','off');
        set(handles.L_Proportional_Ctrl,'Visible','off');
        set(handles.Optimization_Panel,'Visible','off');
        set(handles.Balance_panel,'Visible','off');        
        
        set(handles.R_Torque,'Visible','on');
        set(handles.R_PID,'Visible','off');
        set(handles.R_Adj,'Visible','off');
        set(handles.R_Smoothing,'Visible','off');
        set(handles.R_Proportional_Ctrl,'Visible','off');
        set(handles.Bio_Feedback_panel,'Visible','off');
        
        
% --- Executes on button press in PID.
function PID_Callback(hObject, eventdata, handles)
% hObject    handle to PID (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

        set(handles.L_Torque,'Visible','off');
        set(handles.L_PID,'Visible','on');
        set(handles.L_Adj,'Visible','off');
        set(handles.L_Proportional_Ctrl,'Visible','off');
        set(handles.Optimization_Panel,'Visible','off');
        set(handles.Balance_panel,'Visible','off');
        
        set(handles.R_Torque,'Visible','off');
        set(handles.R_PID,'Visible','on');
        set(handles.R_Adj,'Visible','off');
        set(handles.R_Smoothing,'Visible','off');
        set(handles.R_Proportional_Ctrl,'Visible','off');
        set(handles.Bio_Feedback_panel,'Visible','off');
        
% --- Executes on button press in Adj.
function Adj_Callback(hObject, eventdata, handles)
% hObject    handle to Adj (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

        set(handles.L_Torque,'Visible','off');
        set(handles.L_PID,'Visible','off');
        set(handles.L_Adj,'Visible','on');
        set(handles.L_Proportional_Ctrl,'Visible','off');
        set(handles.Optimization_Panel,'Visible','off');
        set(handles.Balance_panel,'Visible','off');
        
        set(handles.R_Torque,'Visible','off');
        set(handles.R_PID,'Visible','off');
        set(handles.R_Adj,'Visible','on');
        set(handles.R_Smoothing,'Visible','off');
        set(handles.R_Proportional_Ctrl,'Visible','off');
        set(handles.Bio_Feedback_panel,'Visible','off');
        
% --- Executes on button press in Smoothing.
function Smoothing_Callback(hObject, eventdata, handles)
% hObject    handle to Smoothing (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

        set(handles.L_Torque,'Visible','off');
        set(handles.L_PID,'Visible','off');
        set(handles.L_Adj,'Visible','off');
        set(handles.L_Proportional_Ctrl,'Visible','off');
        set(handles.Optimization_Panel,'Visible','off');
        set(handles.Balance_panel,'Visible','off');
        
        set(handles.R_Torque,'Visible','off');
        set(handles.R_PID,'Visible','off');
        set(handles.R_Adj,'Visible','off');
        set(handles.R_Smoothing,'Visible','on');
        set(handles.R_Proportional_Ctrl,'Visible','off');
        set(handles.Bio_Feedback_panel,'Visible','off');
        
        
% --- Executes on button press in Pro_Ctrl.
function Pro_Ctrl_Callback(hObject, eventdata, handles)
% hObject    handle to Pro_Ctrl (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


        set(handles.L_Torque,'Visible','off');
        set(handles.L_PID,'Visible','off');
        set(handles.L_Adj,'Visible','off');
        set(handles.L_Proportional_Ctrl,'Visible','on');
        set(handles.Optimization_Panel,'Visible','off');
        set(handles.Balance_panel,'Visible','off');
        
        set(handles.R_Torque,'Visible','off');
        set(handles.R_PID,'Visible','off');
        set(handles.R_Adj,'Visible','off');
        set(handles.R_Smoothing,'Visible','off');
        set(handles.R_Proportional_Ctrl,'Visible','on');
        set(handles.Bio_Feedback_panel,'Visible','off');
        
        
                
% --- Executes on button press in Optimization.
function Optimization_Callback(hObject, eventdata, handles)
% hObject    handle to Optimization (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

        set(handles.L_Torque,'Visible','off');
        set(handles.L_PID,'Visible','off');
        set(handles.L_Adj,'Visible','off');
        set(handles.L_Proportional_Ctrl,'Visible','off');
        set(handles.Optimization_Panel,'Visible','on');
        set(handles.Balance_panel,'Visible','off');
        
        set(handles.R_Torque,'Visible','off');
        set(handles.R_PID,'Visible','off');
        set(handles.R_Adj,'Visible','off');
        set(handles.R_Smoothing,'Visible','off');
        set(handles.R_Proportional_Ctrl,'Visible','off');
        set(handles.Bio_Feedback_panel,'Visible','off');
        
        
                        
% --- Executes on button press in Balance_Ctrl.
function Balance_Ctrl_Callback(hObject, eventdata, handles)
% hObject    handle to Balance_Ctrl (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

        set(handles.L_Torque,'Visible','off');
        set(handles.L_PID,'Visible','off');
        set(handles.L_Adj,'Visible','off');
        set(handles.L_Proportional_Ctrl,'Visible','off');
        set(handles.Optimization_Panel,'Visible','off');
        set(handles.Balance_panel,'Visible','on');
        
        set(handles.R_Torque,'Visible','off');
        set(handles.R_PID,'Visible','off');
        set(handles.R_Adj,'Visible','off');
        set(handles.R_Smoothing,'Visible','off');
        set(handles.R_Proportional_Ctrl,'Visible','off');
        set(handles.Bio_Feedback_panel,'Visible','off');
        
        
                        
% --- Executes on button press in Bio_Feedback.
function Bio_Feedback_Callback(hObject, eventdata, handles)
% hObject    handle to Bio_Feedback (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

        set(handles.L_Torque,'Visible','off');
        set(handles.L_PID,'Visible','off');
        set(handles.L_Adj,'Visible','off');
        set(handles.L_Proportional_Ctrl,'Visible','off');
        set(handles.Optimization_Panel,'Visible','off');
        set(handles.Balance_panel,'Visible','off');
        
        set(handles.R_Torque,'Visible','off');
        set(handles.R_PID,'Visible','off');
        set(handles.R_Adj,'Visible','off');
        set(handles.R_Smoothing,'Visible','off');
        set(handles.R_Proportional_Ctrl,'Visible','off');
        set(handles.Bio_Feedback_panel,'Visible','on');


% --- Executes on button press in Save_Prop_Prm.
function Save_Prop_Prm_Callback(hObject, eventdata, handles)
% hObject    handle to Save_Prop_Prm (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% 
%         GUI_Variables = handles.GUI_Variables;
%         bt = GUI_Variables.BT; 
%         
%         left_plant_peak_mean = 0; 
%         right_plant_peak_mean = 0; 
%         left_leg_Curr_Combined = 0;
%         right_leg_Curr_Combined = 0;
%         left_leg_fsr_Combined_peak_ref = 0;
%         right_leg_fsr_Combined_peak_ref = 0;
%         left_leg_fsr_Toe_peak_ref = 0;
%         right_leg_fsr_Toe_peak_ref = 0;
%         left_leg_fsr_Heel_peak_ref = 0;
%         right_leg_fsr_Heel_peak_ref = 0;
%         left_leg_torque_calibration_value = 0;
%         right_leg_torque_calibration_value = 0;
% 
% 
%     fwrite(bt,'e');
%           message = fgetl(bt);
%         if message(1) == 83 && message(length(message)-1) == 90 && message(2) == 'P'
%             indexes = find(message==44);
%         left_plant_peak_mean = str2double(message((indexes(1)+1):(indexes(2)-1)));
%         right_plant_peak_mean = str2double(message((indexes(2)+1):(indexes(3)-1)));
%         left_leg_Curr_Combined = str2double(message((indexes(3)+1):(indexes(4)-1)));
%         right_leg_Curr_Combined = str2double(message((indexes(4)+1):(indexes(5)-1)));
%         left_leg_fsr_Combined_peak_ref = str2double(message((indexes(5)+1):(indexes(6)-1)));
%         right_leg_fsr_Combined_peak_ref = str2double(message((indexes(6)+1):(indexes(7)-1)));
%         left_leg_fsr_Toe_peak_ref = str2double(message((indexes(7)+1):(indexes(8)-1)));
%         right_leg_fsr_Toe_peak_ref = str2double(message((indexes(8)+1):(indexes(9)-1)));
%         left_leg_fsr_Heel_peak_ref = str2double(message((indexes(9)+1):(indexes(10)-1)));
%         right_leg_fsr_Heel_peak_ref = str2double(message((indexes(10)+1):(indexes(11)-1)));
%         left_leg_torque_calibration_value = str2double(message((indexes(11)+1):(indexes(12)-1)));
%         right_leg_torque_calibration_value = str2double(message((indexes(12)+1):(indexes(13)-1)));
% 
%         
%         end
%             
%         currDir = cd;       % Current directory
%         saveDir_P = [GUI_Variables.SSID,'_',date,'_Proportional_Parameters'];    % Save directory specific to subject and date
%             savePath_P = [currDir,'\',saveDir_P];    % Save directory specific to subject and date
%         if ~exist(saveDir_P, 'dir')
%             mkdir(currDir, saveDir_P);           % Make a save directory
%         end
%         Filename_P = sprintf('%s_%d.txt',fullfile(savePath_P,[GUI_Variables.SSID,'_',date,'_',...
%             GUI_Variables.TimeStamp,'_','Proportional_Parameters_']),bt.UserData);               %Creates a new filename called "Torque_#"
%                                                                            %Where # is the trial number                                                                           
%         fileID_P = fopen(Filename_P,'w');                                      %Actually creates that file
%         pause(.01);
%         fprintf(fileID_P,['left_plant_peak_mean = ', num2str(left_plant_peak_mean),'\r\n']);
%         fprintf(fileID_P,['right_plant_peak_mean = ', num2str(right_plant_peak_mean),'\r\n']);
%         fprintf(fileID_P,['left_leg_Curr_Combined = ', num2str(left_leg_Curr_Combined),'\r\n']);
%         fprintf(fileID_P,['right_leg_Curr_Combined = ', num2str(right_leg_Curr_Combined),'\r\n']);
%         fprintf(fileID_P,['left_leg_fsr_Combined_peak_ref = ', num2str(left_leg_fsr_Combined_peak_ref),'\r\n']);
%         fprintf(fileID_P,['right_leg_fsr_Combined_peak_ref = ', num2str(right_leg_fsr_Combined_peak_ref),'\r\n']);
%         fprintf(fileID_P,['left_leg_fsr_Toe_peak_ref = ', num2str(left_leg_fsr_Toe_peak_ref),'\r\n']);
%         fprintf(fileID_P,['right_leg_fsr_Toe_peak_ref = ', num2str(right_leg_fsr_Toe_peak_ref),'\r\n']);
%         fprintf(fileID_P,['left_leg_fsr_Heel_peak_ref = ', num2str(left_leg_fsr_Heel_peak_ref),'\r\n']);
%         fprintf(fileID_P,['right_leg_fsr_Heel_peak_ref = ', num2str(right_leg_fsr_Heel_peak_ref),'\r\n']);
%         fprintf(fileID_P,['left_leg_torque_calibration_value = ', num2str(left_leg_torque_calibration_value),'\r\n']);
%         fprintf(fileID_P,['right_leg_torque_calibration_value = ', num2str(right_leg_torque_calibration_value),'\r\n']);
%         fclose(fileID_P);
%         
%         
%    

% --- Executes on button press in Load_Prop_Prm.
function Load_Prop_Prm_Callback(hObject, eventdata, handles)
% hObject    handle to Load_Prop_Prm (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

GUI_Variables = handles.GUI_Variables;
bt = GUI_Variables.BT; 

if (bt.Status=="open")
   
    try
       
    [filename, pathname] = uigetfile({'*.txt', 'Matlab Files (*.txt)'; ...
                '*.*',                   'All Files (*.*)'});

%This code checks if the user pressed cancel on the dialog.
        if isequal(filename,0) || isequal(pathname,0)
             disp('User pressed cancel')
        else
             disp(['User selected ', fullfile(pathname, filename)])

        end
Old_Pro_Prm = importdata(fullfile(pathname, filename));

fwrite(bt,'g'); %send the character "%"

for i=1:length(Old_Pro_Prm.data)
    fwrite(bt,Old_Pro_Prm.data(i),'double');
    disp(['Old Proportional Parameters ','[',num2str(i),'] ',num2str(Old_Pro_Prm.data(i))]);
   % pause(0.1);
end
    catch

    end
end


% --- Executes when selected object is changed in Prop_Ctrl_Panel.
function Prop_Ctrl_Panel_SelectionChangedFcn(hObject, eventdata, handles)
% hObject    handle to the selected object in Prop_Ctrl_Panel 
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

GUI_Variables = handles.GUI_Variables;
bt = GUI_Variables.BT;


value_Pivot = get(handles.Activate_Prop_Pivot,'value');
value_ID = get(handles.Activate_Prop_ID,'value');

if (bt.Status=="open")

    try
     

        if (value_Pivot == 1)         
                fwrite(bt,'#');
                disp('Pivot Prop Ctrl'); 
                set(handles.Take_Baseline,'Enable','on');
               
        elseif (value_ID == 1)
                fwrite(bt,'c');
                disp('ID Prop Ctrl');
                set(handles.Take_Baseline,'Enable','on');
            
        end
            
            axes(handles.PROP_FUNCTION_axes);
            x=0.4:0.01:1.2;
            plot(x,GUI_Variables.Setpoint*(128.1*x.^2-50.82*x+22.06)/(128.1-50.82+22.06));

        
               
    catch
       
    end
end



% --- Executes on button press in Activate_Prop_Ctrl.
function Activate_Prop_Ctrl_Callback(hObject, eventdata, handles)
% hObject    handle to Activate_Prop_Ctrl (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



GUI_Variables = handles.GUI_Variables;
bt = GUI_Variables.BT;

 str = get(handles.Activate_Prop_Ctrl,'string');
% 
%             set(handles.Activate_Prop_Pivot,'enable','on');   % TN 04-26-2019
%             set(handles.Activate_Prop_ID,'enable','on');      % TN 04-26-2019

      
if (bt.Status=="open")

    try  
     

        if strcmp( str, 'Activate Prop Control' )
            GUI_Variables.PropOn = 1; 
            fwrite(bt,'l');
            disp( 'Activate Prop Control' );
            set(handles.Activate_Prop_Ctrl,'string','Deactivate Prop Control');
%            set(handles.Prop_Ctrl_Panel,'visible','on');
%             set(handles.Activate_Prop_Pivot,'value',0);
%             set(handles.Activate_Prop_ID,'value',0);
%            set(handles.Take_Baseline,'enable','on');
%            set(handles.Check_Baseline,'enable','on');
            set(handles.Start_ATP,'Enable','on');
            set(handles.Stop_ATP,'Enable','on');
        else
            GUI_Variables.PropOn = 0;
            disp( 'Deactivate Prop Control' );
            fwrite(bt,'^');
            set(handles.Activate_Prop_Ctrl,'string','Activate Prop Control');
%            set(handles.Prop_Ctrl_Panel,'visible','off');
%             set(handles.Activate_Prop_Pivot,'value',0);
%             set(handles.Activate_Prop_ID,'value',0);
%            set(handles.Take_Baseline,'enable','off');
%            set(handles.Check_Baseline,'enable','off');
            set(handles.Start_ATP,'Enable','off');
            set(handles.Stop_ATP,'Enable','off');
        
            axes(handles.PROP_FUNCTION_axes);
            clf(h);
            x=0.4:0.01:1.2;
            plot(x,x*0);
        
        
    
        end
    catch
    end
end
handles.GUI_Variables = GUI_Variables;
guidata(hObject,handles);

% --- Executes on button press in Start_Timer.
function Start_Timer_Callback(hObject, eventdata, handles)
% hObject    handle to Start_Timer (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
 
str = get(handles.Start_Timer,'string');
a = clock;
time = a(4)*360+a(5)*60+a(6);
Send_Trig_Callback(hObject, 0, handles);

if strcmp(get(handles.Start_Trial,'Enable'), 'off')
    if strcmp(str,'Start')
        set(handles.Start_Timer,'string','Pause');
        set(handles.Split_Timer,'enable','on');
        set(handles.Reset_Timer,'enable','off');
        start_time = time;
        setappdata(handles.Start_Timer,'start_time',start_time);
    else
        set(handles.Start_Timer,'string','Start');
        set(handles.Reset_Timer,'enable','on');
        set(handles.Split_Timer,'enable','off');
    end
end
 
% --- Executes on button press in Reset_Timer.
function Reset_Timer_Callback(hObject, eventdata, handles)
% hObject    handle to Reset_Timer (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.Split_Timer,'enable','off');
set(handles.Start_Timer,'enable','on');
set(handles.Reset_Timer,'enable','off');
set(handles.Lap_Timer,'string','0');
set(handles.Timer_Value,'string','0');
set(handles.Start_Timer,'String','Start');

% --- Executes on button press in Split_Timer.
function Split_Timer_Callback(hObject, eventdata, handles)
% hObject    handle to Split_Timer (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
a = clock;
stop_time = a(4)*360 + a(5)*60 + a(6);
%guidata(handles.Split_Timer,stop_time);
start_time = getappdata(handles.Start_Timer,'start_time');
setappdata(handles.Start_Timer,'start_time',stop_time);
split_time = stop_time - start_time;
Send_Trig_Callback(hObject, 0, handles);

b = get(handles.Lap_Timer,'string');
c = str2double(b);
c = c + 1;
set(handles.Lap_Timer,'string',num2str(c));
set(handles.Timer_Value,'string',sprintf('%.3f',split_time));


GUI_Variables = handles.GUI_Variables;
bt = GUI_Variables.BT;

% if c == 1
%     bt.UserData = bt.UserData + 1;
% end

currDir = cd;       % Current directory
saveDir = [GUI_Variables.SSID,'_',date];
savePath = [currDir,'\',saveDir];    % Save directory specific to subject and date
if ~exist(saveDir, 'dir')
    mkdir(currDir, saveDir);           % Make a save directory
end
Filename = sprintf('%s_%d.txt',fullfile(savePath,[GUI_Variables.SSID,'_',date,'_',GUI_Variables.TimeStamp,'_',...
    'Timer_']),bt.UserData); 
        
fileID = fopen(Filename,'a');

fprintf(fileID,'Lap %d: %4.3f s\r\n',c,split_time);
fclose(fileID);



% --- Executes on button press in Start_ATP.
function Start_ATP_Callback(hObject, eventdata, handles)
% hObject    handle to Start_ATP (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

GUI_Variables = handles.GUI_Variables;
bt = GUI_Variables.BT; 
% 
if bt.Status=="open"

 
      
    RLCount_start = GUI_Variables.RLCount;
    LLCount_start = GUI_Variables.LLCount;
    start_time = min(RLCount_start,LLCount_start);
    setappdata(handles.Start_ATP,'start_time',start_time);
%     disp('Start');
%     disp(num2str(start_time));
    set(handles.Start_ATP,'enable','off');
    set(handles.Stop_ATP,'enable','on');
end


   
% --- Executes on button press in Stop_ATP.
function Stop_ATP_Callback(hObject, eventdata, handles)
% hObject    handle to Stop_ATP (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

GUI_Variables = handles.GUI_Variables;
bt = GUI_Variables.BT; 

if bt.Status=="open"
    GUI_Variables = handles.GUI_Variables;
    RLCount_stop = GUI_Variables.RLCount;
    LLCount_stop = GUI_Variables.LLCount;
    stop_time = max(RLCount_stop,LLCount_stop);
%    setappdata(handles.Start_ATP,'stop_time',stop_time); 
    start_time = getappdata(handles.Start_ATP,'start_time');
  
if stop_time > start_time
    
        LLTRQ = GUI_Variables.LLTRQ;
        LLFSR = GUI_Variables.LLFSR;
        RLTRQ = GUI_Variables.RLTRQ;
        RLFSR = GUI_Variables.RLFSR;
        
        LLTRQ = LLTRQ(start_time:stop_time - 1);
        LLFSR = LLFSR(start_time:stop_time - 1); 
        RLTRQ = RLTRQ(start_time:stop_time - 1);
        RLFSR = RLFSR(start_time:stop_time - 1);
        
        dt = .01; % Since the Arduino is set to send values every 10 ms, dt is .01 S
        t = 1: length(RLTRQ);                                                %Creates a time Vector equal in length to the number of Torque Values Recieved
        t = t .* dt; % Scales the time Vector, knowing how often Arduino sends values,
    
    %data = [t,LLTRQ,LLFSR,RLTRQ,RLFSR];
    
        currDir = cd;       % Current directory
        saveDir = [GUI_Variables.SSID,'_',date];
        savePath = [currDir,'\',saveDir];    % Save directory specific to subject and date
        if ~exist(saveDir, 'dir')
            mkdir(currDir, saveDir);           % Make a save directory
        end
        Filename = sprintf('%s_%d.txt',fullfile(savePath,[GUI_Variables.SSID,'_',date,'_',GUI_Variables.TimeStamp,'_',...
            'ATP_']),bt.UserData); 
        
        fileID = fopen(Filename,'w');
        fprintf(fileID,'%6.2f %6.2f %6.2f %6.2f %6.2f \n',[t; RLTRQ; RLFSR; LLTRQ; LLFSR]);
        fclose(fileID);
%        bt.UserData = bt.UserData + 1;
end
    
    set(handles.Start_ATP,'enable','on');
    set(handles.Stop_ATP,'enable','off');
end



% --- Executes on button press in Send_ATP.
function Send_ATP_Callback(hObject, eventdata, handles)
% hObject    handle to Send_ATP (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


GUI_Variables = handles.GUI_Variables;
bt = GUI_Variables.BT; 

if (bt.Status=="open")
   
       
    [filename, pathname] = uigetfile({'*.mat', 'Matlab Files (*.mat)'; ...
                '*.*',                   'All Files (*.*)'});

%This code checks if the user pressed cancel on the dialog.
        if isequal(filename,0) || isequal(pathname,0)
             disp('User pressed cancel')
        else
             disp(['User selected ', fullfile(pathname, filename)])

        end
        
ATP_file = fullfile(pathname, filename);

%fwrite(bt,'g'); %send the character "%"

[right_torque,left_torque]=averaging_torque(ATP_file);
% for i=1:101
% if left_torque(i) < 0
%     left_torque(i) = 0;
% end
% end
% 
% for i=1:101
% if right_torque(i) > 0
%     right_torque(i) = 0;
% end
% end

%close all;
figure();
subplot(2,1,1)
plot(0:100,left_torque);title('Averaged left torque profile');
subplot(2,1,2)
plot(0:100,right_torque);title('Averaged right torque profile');
   
ATP = [right_torque,left_torque];
% Arrange it to comma-separated string
str = num2str(ATP);
str = regexprep(str,'\s+',',');
% Save as 'sample.h' file
cur_d = cd;
new_file = strrep(cur_d,'Gui','Exo\ATP.h');
fid = fopen(new_file,'w');
fprintf(fid,'double ATP[202] = {%s};\n',str);
fclose(fid);

end



% --- Executes on button press in ATP_Mode.
function ATP_Mode_Callback(hObject, eventdata, handles)
% hObject    handle to ATP_Mode (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
GUI_Variables = handles.GUI_Variables;
bt = GUI_Variables.BT; 

if (bt.Status=="open")
   
fwrite(bt,'j');        %TN

            set(handles.Activate_Prop_Ctrl,'string','Activate Prop Control');
%             set(handles.Prop_Ctrl_Panel,'visible','off');
            set(handles.Activate_Prop_Pivot,'value',0);
            set(handles.Activate_Prop_ID,'value',0);
            set(handles.Take_Baseline,'enable','off');
            set(handles.Start_ATP,'Enable','off');
            set(handles.Stop_ATP,'Enable','off');
end



