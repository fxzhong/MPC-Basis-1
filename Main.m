clear all;
close all;
%first initialize all states
%initialize all parameters
%construct system model (FK)
%Define objective function and constraints
%Solve the optimization and gets the open-loop optimal control input
%update the system states and re-initialize the optimization

%%Other initialisation for simulation
width = 800;
height = 600;
billy = figure('Position', [300, 100, width, height]);
figHandle = billy;
%set up the movie.
writerObj = VideoWriter('ZC','MPEG-4'); % Name it.
writerObj.FrameRate = 30; % How many frames per second.
open(writerObj);

%conrol time horizon
t_h = 30;
%control task specification
x_obst = [0.3*width, 0.5*height-50, 80]; %the size is in radius
x_start = [50, 0.5*height];
x_end = [0.5*width, 0.4*height];

x0 = x_start;
x_current = x0;
x = zeros(21,2);
%initialize traj to enable immediate trajectory curve drawing
x_traj_start = x_start;
x_traj_end = x_start;
%Initialize control input vectors
u0 = zeros(t_h, 2);
u0(:,1) = 2;
u0(:,2) = 2;
u = u0;

t = 0;
obj = 0;
obj_old = 0;
d_obj = 1000;

while true

    while abs(d_obj) > 5
        %Calculate the initial estimated states evolution from t1 to tn
        x(1,:) = x_current;
        for i = 1:t_h-1
            x(i+1,:) = x(i,:) + u(i,:);
        end

        %Caculate costate Lambda from Lagrangian Multiplier from tn back to t1
        phi = 0.5*norm(x(t_h,:) - x_end)^2;
        lambda = zeros(t_h, 2);
        lambda(t_h, :) = -0.002*(x(t_h,:) - x_end);
        for i = t_h-1:-1:1
            %lambda(i) = lambda(i+1) + dH/dx
            lambda(i, :) = lambda(i+1, :) + 0.5*(x(i,:)-x_obst(1:2))/(norm(x(i,:)-x_obst(1:2))- x_obst(3))/(norm(x(i,:)-x_obst(1:2)) - x_obst(3))/norm(x(i,:)-x_obst(1:2));
        end

        u = u + lambda;
    
        %Calculate objective function with the control horizon
        obj = phi;
        for i = 1:t_h
            obj = obj + 1/(norm(x(i,:) - x_obst(1:2)) - x_obst(3));
        end
        d_obj = obj - obj_old;
        obj_old = obj;
    end
    
    %Plot trivial things
    plot(x_obst(1), x_obst(2), '.k', 'MarkerSize', 4*x_obst(3));hold on;
    plot(x_start(1), x_start(2), 'ok', 'MarkerSize',10,'MarkerFaceColor','g');
    plot(x_end(1), x_end(2), 'ok', 'MarkerSize',10,'MarkerFaceColor','r');
    plot([x_traj_start(:,1), x_traj_end(:,1)], [x_traj_start(:,2), x_traj_end(:,2)],'-','Color','r','LineWidth',3);      
    %Plot variables
    plot(x_current(1), x_current(2), '.r', 'MarkerSize', 5);
    plot(x(:,1), x(:,2), '.b', 'MarkerSize', 5);
    textt = ['Obj: ', num2str(obj)];
    text(30,30,textt,'Color','k');
    textt = ['dObj: ', num2str(d_obj)];
    text(30,60,textt,'Color','k');
    %Keyboard input callback
    if strcmpi(get(billy,'CurrentKey'),'q')
        close(writerObj);
        objects = imaqfind; %find video input objects in memory
        delete(objects); %delete a video input object from memory
        close all;
        break;
    end
    
    grid on;
    axis equal;    
    xlim([0 width]);
    set(gca,'XTick',0:100:width);
    ylim([0 height]);
    set(gca,'YTick',0:100:height);
    visual = sprintf('Step: %d, %d', t, obj);
    title(visual);
    hold off;
    
    %Video settings
    frame = getframe(gca); % 'gcf' can handle if you zoom in to take a movie.
    writeVideo(writerObj, frame);
    
    t = t + 1;

    x_current = x_current + u(1,:);
    
    %delete the endpoint flag
%     x_traj_start = x_traj_start(1:end-1,:);
%     x_traj_end = x_traj_end(1:end-1,:);
    %push the newest trajectory point
    x_traj_start(end+1,:) = x_traj_end(end,:);
    x_traj_end(end+1,:) = x_current;
    %push the endpoint flag again
%     x_traj_start(end+1) = x_start(1);
%     x_traj_start(end+1) = x_start(2);
%     x_traj_end_x(end+1) = x_start(1);
%     x_traj_end_y(end+1) = x_start(2);
    %reset optimization-related variables
    d_obj = 1000;
    u = u0;
   
    pause(.01); 
end

close all;
close(writerObj);