function [msg, data] = get_message()

    global GUI_Variables
    bt = GUI_Variables.BT;
    message = fgetl(bt);
    msg = message(2);
    if message(1) == 83 && message(length(message)-1) == 90
        indexes = find(message==',');
        data=zeros(1,length(indexes)-1);
        for index_iterator = 1:(length(indexes)-1)
            data(index_iterator) = str2double(message((indexes(index_iterator)+1):(indexes(index_iterator+1)-1)));
        end
        data = data ./ 100;
    end