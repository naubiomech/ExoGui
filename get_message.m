function [msg, data] = get_message(bt)

    msg = '\0';
    data = [];
    count = 0;
    
    header = fread(bt, 3)';
    
    if length(header) == 3 && header(1) == 'S'
        msg = header(2);
        count = header(3);
        if count > 0
            data = fread(bt, count, 'float')';
        end
    end
    
    if count ~= length(data)
        data = NaN(count);
    end