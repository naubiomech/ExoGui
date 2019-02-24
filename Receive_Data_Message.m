function GUI_Variables = Receive_Data_Message(GUI_Variables, handles)
    [msg, Data] = get_message(GUI_Variables.BT);
    GUI_Variables = command(GUI_Variables, msg, Data, handles);
end
