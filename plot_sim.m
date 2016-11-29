function plot_sim(arm)
    close all

    % name the whole window and define the mouse callback function
    f = figure;
    set(f,'WindowButtonMotionFcn','','WindowButtonDownFcn',@click_down);
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
    axis([-3.5 3.5 -3.6 3.6]);

    % dots for the hinges
    h1 = plot(0,0,'.k','MarkerSize',20); % first link anchor
    h2 = plot(0,0,'.k','MarkerSize',20); % link1 to link2 hinge

    % timer label
    timer = text(-3.2,-3.2,'0.00','FontSize',10);

    % torque meters
    tmeter1 = text(0.4,-3.2,'0.00','FontSize',10,'Color', 'r');
    tmeter2 = text(2.0,-3.2,'0.00','FontSize',10,'Color', 'b');

    % target point
    targetPt = plot(arm.xtarget,arm.ytarget,'xg','MarkerSize',15,'LineWidth',1.5);

    %%%% new goal location %%%%
    x_goal = -1;
    y_goal = 1.5
    ee_goal = inv_kin(arm.l1,arm.l2,x_goal,y_goal);

    % goal point
    goalPt = plot(x_goal,y_goal,'xr','MarkerSize',15,'LineWidth',1.5);

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

    epsilon = 0.001;
    tic % start the clock
    while (ishandle(f))
        figData = get(f,'UserData');

        tnew = toc;
        dt = tnew - told;

        %Old velocity and position
        xold = [z1(1),z1(3)];
        vold = [z1(2),z1(4)];

        % call RHS given old state
        [zdot1, T1, T2] = full_dynamics(tnew,z1,arm);
        vinter1 = [zdot1(1),zdot1(3)];
        ainter = [zdot1(2),zdot1(4)];

        vinter2 = vold + ainter*dt; % update velocity based on old RHS call

        % update position and velocity
        xnew = xold + vinter2*dt;
        vnew = (xnew-xold)/dt;

        z2 = [xnew(1) vnew(1) xnew(2) vnew(2)];

        z1 = z2;
        told = tnew;

        arm_pos = forward_kin(arm.l1,arm.l2,z1(1),z1(3))
        armx = arm.xtarget
        army = arm.ytarget
        %x_dist = abs(arm_pos(1)-arm.xtarget)
        %y_dist = abs(arm_pos(2)-arm.ytarget)
    %     if abs(arm_pos(1)-arm.xtarget) <= epsilon && abs(arm_pos(2)-arm.ytarget)<= epsilon
    %         arm.xtarget = x_goal;
    %         arm.ytarget = y_goal;
    %         
    %         set(targetPt,'xData',arm.xtarget); % change the target point graphically
    %         set(targetPt,'yData',arm.ytarget);
    %     end
        %%%%%%%%%%%%%%%%%%%%

        %If there are new mouse click locations, then set those as the new
        %target.
        if ~isempty(figData.xtarget)
        arm.xtarget = figData.xtarget;
        end

        if ~isempty(figData.ytarget)
        arm.ytarget = figData.ytarget;
        end
        set(targetPt,'xData',arm.xtarget); %Change the target point graphically.
        set(targetPt,'yData',arm.ytarget);


        % save current time 
        tstar = told; 

        % show time on screen 
        set(timer,'string',strcat('time: ',num2str(tstar,3),'s'))

        curr_theta1 = z1(1);
        curr_theta2 = z1(3);

        
        
        % rotation matrices to manipulate the vertices of the patch objects
        % using theta1 and theta2 from the output state vector
        rot1 = [cos(curr_theta1), -sin(curr_theta1); 
                sin(curr_theta1), cos(curr_theta1)];
        rot1 = rot1*[xdat1;ydat1];

        set(link1,'xData',rot1(1,:))
        set(link1,'yData',rot1(2,:))

        rot2 = [cos(curr_theta2+curr_theta1), -sin(curr_theta2+curr_theta1); 
                sin(curr_theta2+curr_theta1),cos(curr_theta2+curr_theta1)];
        rot2 = rot2*[xdat2;ydat2];

        % add the midpoint of the far edge of the first link to all points in link 2
        set(link2,'xData',rot2(1,:)+(rot1(1,3)+rot1(1,4))/2) 
        set(link2,'yData',rot2(2,:)+(rot1(2,3)+rot1(2,4))/2)

        % change the hinge dot location
        set(h2,'xData',(rot1(1,3)+rot1(1,4))/2)
        set(h2,'yData',(rot1(2,3)+rot1(2,4))/2)

        % show torques on screen 
        set(tmeter1,'string',strcat('tau1: ', num2str(T1,2),' Nm'));
        set(tmeter2,'string',strcat('tau2: ', num2str(T2,2),' Nm'));

        drawnow;
    end
end

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