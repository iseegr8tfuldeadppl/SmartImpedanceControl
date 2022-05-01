function plot_sim(arm)
    close all

    % name the whole window and define the mouse callback function
    f = figure;
    set(f,'WindowButtonMotionFcn','','WindowButtonUpFcn',@click_up,'WindowButtonDownFcn',@click_down,'KeyPressFc',@key_press);
    %set(f,'WindowButtonMotionFcn','','WindowButtonDownFcn',@click_down,'KeyPressFc',@key_press);
    set(gcf,'Visible', 'off'); 

    figData.xtarget = [];
    figData.ytarget = [];
    figData.Fx = [];
    figData.Fy = [];
    figData.xend = [];
    figData.yend = [];
    figData.fig = f;
    figData.tarControl = true;

    % pendulum animation subplot
    figData.simArea = subplot(1,1,1); 
    axis equal
    hold on

    % create link1 object
    width1 = arm.l1*0.05;
    xdat1 = 0.5*width1*[-1 1 1 -1];
    ydat1 = arm.l1*[0 0 1 1];
    link1 = patch(xdat1, ydat1, [0 0 0 0], 'r');

    % create link2 object
    width2 = arm.l2*0.05;
    xdat2 = 0.5*width2*[-1 1 1 -1];
    ydat2 = arm.l2*[0 0 1 1];
    link2 = patch(xdat2, ydat2, [0 0 0 0], 'b');
    
    % set axis bounds
    axis([-3 3 -3 3]);

    % dots for the hinges
    h1 = plot(0,0,'.k','MarkerSize',20); % first link anchor
    h2 = plot(0,0,'.k','MarkerSize',20); % link1 to link2 hinge
    h3 = plot(0,0,'.k','MarkerSize',20); % end-effector 

    % timer label
    timer = text(-2.7,-2.7,'0.00','FontSize',10);

    % torque meters
    tmeter1 = text(0.4,-2.7,'0.00','FontSize',10,'Color', 'r');
    tmeter2 = text(1.8,-2.7,'0.00','FontSize',10,'Color', 'b');

    %%%% get list of new goal locations %%%%
    xy_goals = [-1, 1.5;
                -1, 1;
                -1, 0.5;
                -1, 0;
                -1, -0.5];
    num_waypts = 5;
    for i = 1:num_waypts
        plot(xy_goals(i,1),xy_goals(i,2),'xk','MarkerSize',10,'LineWidth',1.5);
    end
        
    % target point
    targetPt = plot(arm.xtarget,arm.ytarget,'xr','MarkerSize',10,'LineWidth',1.5);
    
    % goal point
    % goalPt = plot(xy_goals(4,1),xy_goals(4,2),'xr','MarkerSize',15,'LineWidth',1.5);

    hold off

    % make the whole window big for handy viewing
    set(f, 'units', 'inches', 'position', [5 5 5 5])
    set(f, 'Color',[1,1,1]);

    % turn the axis off
    grid on
    ax = get(f,'Children');
    %set(ax,'Visible','off');

    % move gui to center of screen
    movegui(f,'center')
    set(gcf,'Visible', 'on'); 

    % get current state of arm
    z1 = arm.init;
    told = 0;

    set(f,'UserData',figData);

    epsilon = 0.1;
    idx = 1;
    user_click = false;
    % start the clock
    tic
    while (ishandle(f))
        figData = get(f,'UserData');

        % get current time
        tnew = toc;
        dt = tnew - told;

        % old velocity and position
        xold = [z1(1),z1(3)];
        vold = [z1(2),z1(4)];

        % update state and torques based on dynamics
        [zdot1, T1, T2] = full_dynamics(tnew,z1,arm);
        vinter1 = [zdot1(1),zdot1(3)];
        ainter = [zdot1(2),zdot1(4)];

        % update velocity 
        vinter2 = vold + ainter*dt; 

        % update position and velocity
        xnew = xold + vinter2*dt;
        vnew = (xnew-xold)/dt;

        % construct new state vector
        z2 = [xnew(1) vnew(1) xnew(2) vnew(2)];

        z1 = z2;
        told = tnew;

        arm_pos = forward_kin(arm.l1,arm.l2,z1(1),z1(3))
        x_dist = abs(arm_pos(1)-arm.xtarget);
        y_dist = abs(arm_pos(2)-arm.ytarget);
        if x_dist <= epsilon && y_dist <= epsilon
            arm.xtarget = xy_goals(idx,1);
            arm.ytarget = xy_goals(idx,2);
       
            % plot new target point
            set(targetPt,'xData',arm.xtarget); 
            set(targetPt,'yData',arm.ytarget);
            if idx < num_waypts && user_click == false
                idx = idx + 1;
            elseif user_click == false
                idx = 1;
            else
                user_click = false;
            end
        end
        
        %%%%%%%%%%%%%%%%%%%%

        %If there are new mouse click locations, then set those as the new
        %target.
        if ~isempty(figData.xtarget)
            user_click = true;
            arm.xtarget = figData.xtarget;
        end

        if ~isempty(figData.ytarget)
            user_click = true;
            arm.ytarget = figData.ytarget;
        end
        set(targetPt,'xData',arm.xtarget); % change the target point graphically.
        set(targetPt,'yData',arm.ytarget);

        ee_loc = forward_kin(arm.l1,arm.l2,z1(1),z1(3));
        figData.xend = ee_loc(1);
        figData.yend = ee_loc(2);
        set(f,'UserData',figData);

        % if got a force at the end-effector
        if ~isempty(figData.Fx)
            arm.Fx = figData.Fx;
        end
        if ~isempty(figData.Fy)
            arm.Fy = figData.Fy;
        end
        
        % save current time 
        tstar = told; 

        % show time on screen 
        set(timer,'string',strcat('time: ',num2str(tstar,3),'s'))

        curr_theta1 = z1(1);
        curr_theta2 = z1(3);

        % rotation matrices to manipulate the vertices of the patch objects
        % using theta1 and theta2 from the output state vector
        
        % TODO: THIS IS A HACK BUT WHY????
        neg90_rot = [cos(-pi/2), -sin(-pi/2); 
                sin(-pi/2), cos(-pi/2)];
            
        rot1 = [cos(curr_theta1), -sin(curr_theta1); 
                sin(curr_theta1), cos(curr_theta1)];
        rot1 = neg90_rot*rot1*[xdat1;ydat1];

        set(link1,'xData',rot1(1,:))
        set(link1,'yData',rot1(2,:))

        rot2 = [cos(curr_theta2+curr_theta1), -sin(curr_theta2+curr_theta1); 
                sin(curr_theta2+curr_theta1),cos(curr_theta2+curr_theta1)];
        rot2 = neg90_rot*rot2*[xdat2;ydat2];

        % add the midpoint of the far edge of the first link to all points in link 2
        set(link2,'xData',rot2(1,:)+(rot1(1,3)+rot1(1,4))/2) 
        set(link2,'yData',rot2(2,:)+(rot1(2,3)+rot1(2,4))/2)

        % change the hinge dot location
        set(h2,'xData',(rot1(1,3)+rot1(1,4))/2)
        set(h2,'yData',(rot1(2,3)+rot1(2,4))/2)
        
%         xy_curr = inv_kin(arm.l1, arm.l2, curr_theta1, curr_theta2)
%         set(h3,'xData',xy_curr(1))
%         set(h3,'yData',xy_curr(2))

        % show torques on screen 
        set(tmeter1,'string',strcat('tau1: ', num2str(T1,2),' Nm'));
        set(tmeter2,'string',strcat('tau2: ', num2str(T2,2),' Nm'));

        drawnow;
    end
end

% When click-up occurs, disable the mouse motion detecting callback
% function click_up(varargin)
%     figData = get(varargin{1},'UserData');
%     set(figData.fig,'WindowButtonMotionFcn','');
%     figData.Fx = 0;
%     figData.Fy = 0;
%     set(varargin{1},'UserData',figData);
% end

% when click-down occurs, enable the mouse motion detecting callback
function click_down(varargin)
    figData = get(varargin{1},'UserData');
    figData.Fx = 0;
    figData.Fy = 0;

    set(figData.fig,'WindowButtonMotionFcn',@mouse_pos);
    set(varargin{1},'UserData',figData);
end

% checks mouse position and sends it back up
function mouse_pos(varargin)
    figData = get(varargin{1},'UserData');

    mousePos = get(figData.simArea,'CurrentPoint');
    if figData.tarControl
        figData.xtarget = mousePos(1,1);
        figData.ytarget = mousePos(1,2);
    else
        figData.Fx = 10*(mousePos(1,1)-figData.xend);
        figData.Fy = 10*(mousePos(1,2)-figData.yend);
    end
     set(varargin{1},'UserData',figData);
end

% any keypress switches from dragging the setpoint to applying a
% disturbance.
function key_press(hObject, eventdata, handles)

figData = get(hObject,'UserData');

figData.tarControl = ~figData.tarControl;

    if figData.tarControl
       disp('Mouse will change the target point of the end effector.')
    else
       disp('Mouse will apply a force on end effector.') 
    end
set(hObject,'UserData',figData);
end
