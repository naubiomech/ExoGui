function [msg, data] = get_message()

    global GUI_Variables
    bt = GUI_Variables.BT;
    msg = '\0';
    data = [];
    count = 0;
    
    header = fread(bt, 3)';
    
    if length(header) == 3 && header(1) == 'S'
        msg = header(2);
        count = header(3);
        data = fread(bt, count, 'float')';
    end
    
    if count ~= length(data)
        msg = '\0';
        data = [];
    end