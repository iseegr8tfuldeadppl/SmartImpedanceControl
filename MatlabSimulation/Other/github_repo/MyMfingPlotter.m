f=figure;

set(f,'WindowButtonDown',{@sendout});

function sendout(varargin)
    set(f,'UserData',1); 
    y =get(f,'UserData');
    display(y);
end