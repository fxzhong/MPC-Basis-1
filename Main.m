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
t_h = 50;
%control task specification
x_obst = [0.3*width, 0.5*height-50, 80]; %the size is in radius
x_start = [50, 0.5*height];
x_end = [0.9*width, 0.4*height];

x0 = x_start;
x_current = x0;
x = zeros(21,2);
%initialize traj to enable immediate trajectory curve drawing
x_traj_start = x_start;
x_traj_end = x_start;
%Initialize control input vectors
u0 = zeros(t_h, 2);
u0(:,1) = 5;
u0(:,2) = 5;
u = u0;
dxm = 6;

t = 0;
t_step = 0;
obj = 0;
obj_old = 0;
d_obj = 1000;
%preset parameters for online tuning
du_k = 1;
d_obj_th = 5;
weight = 2;

FrameRate = 0;
elapsedTime = 0;

while true
    
    tic
    %x_end = [0.9*width, 0.4*height + 50*sin(0.1*t)];
    while abs(d_obj) > d_obj_th
        %Calculate the initial estimated states evolution from t1 to tn
        x(1,:) = x_current;
        for i = 1:t_h-1
            x(i+1,:) = x(i,:) + u(i,:);
        end

        %Caculate costate Lambda from Lagrangian Multiplier from tn back to t1
        phi = 0.5*norm(x(t_h,:) - x_end)^2;
        lambda = zeros(t_h, 2);
        lambda(t_h, :) = -0.001*(x(t_h,:) - x_end);
        for i = t_h-1:-1:1
            %lambda(i) = lambda(i+1) + dH/dx
            lambda(i,:)  = lambda(i+1,:) + weight*(x(i+1,:)-x_obst(1:2))/(norm(x(i+1,:)-x_obst(1:2))- x_obst(3))/(norm(x(i+1,:)-x_obst(1:2)) - x_obst(3))/norm(x(i+1,:)-x_obst(1:2));
        end
        
        %Calculate objective function with the control horizon
        obj = phi;
        du = zeros(t_h, 2);
        dxmax = 0;
        for i = 1:t_h-1
            obj = obj + 1/(norm(x(i,:) - x_obst(1:2)) - x_obst(3));
            du(i,:) = lambda(i,:);
            weight_u = 0.1;
            if norm(x(i+1,:)-x(i,:)) > dxm
                obj = obj + 0.5*(norm(x(i+1,:)-x(i,:))-dxm)^2;
                du(i,:) = du(i,:) - weight_u*(norm(x(i+1,:)-x(i,:))-dxm)*(x(i+1,:)-x(i,:));
            end
            if norm(x(i+1,:)-x(i,:)) > dxmax
                dxmax = norm(x(i+1,:)-x(i,:));
            end
        end
        
        u = u + du_k*du;
            
        d_obj = obj - obj_old;
        obj_old = obj;
        t_step = t_step + 1;
    end

    %Plot trivial things
    plot(x_obst(1), x_obst(2), '.k', 'MarkerSize', 4*x_obst(3));hold on;
    plot(x_start(1), x_start(2), 'ok', 'MarkerSize',5,'MarkerFaceColor','g');
    plot(x_end(1), x_end(2), 'ok', 'MarkerSize',5,'MarkerFaceColor','r');
    plot([x_traj_start(:,1), x_traj_end(:,1)], [x_traj_start(:,2), x_traj_end(:,2)],'-','Color','r','LineWidth',2);      
    %Plot variables
    plot(x_current(1), x_current(2), '.r', 'MarkerSize', 5);
    plot(x(:,1), x(:,2), '.b', 'MarkerSize', 5);
    textt = ['Obj: ', num2str(obj)];
    text(30,30,textt,'Color','k');
    textt = ['dObj: ', num2str(d_obj)];
    text(30,60,textt,'Color','k');
    textt = ['Opti_t: ', num2str(t_step)];
    text(30,90,textt,'Color','k');
    textt = ['vtmax: ', num2str(dxmax)];
    text(30,120,textt,'Color','k');
    textt = ['du-k: ', num2str(du_k)];
    text(30,height-30,textt,'Color','k');
    textt = ['dObj-th: ', num2str(d_obj_th)];
    text(30,height-60,textt,'Color','k');
    textt = ['weight: ', num2str(weight)];
    text(30,height-90,textt,'Color','k');
    textt = [num2str(FrameRate), ' fps'];
    text(width-110,30,textt,'Color','k');
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
%     frame = getframe(gca); % 'gcf' can handle if you zoom in to take a movie.
%     writeVideo(writerObj, frame);
    
    t = t + 1;
    
    t_step = 0;
    %only apply the first controller input to the final execution 
    x_current = x_current + u(1,:);
    
    %push the newest trajectory point
    x_traj_start(end+1,:) = x_traj_end(end,:);
    x_traj_end(end+1,:) = x_current;
    %push the endpoint flag again
    %reset optimization-related variables
    d_obj = 1000;
    u = u0;
   
    pause(.01); 
    
    elapsedTime = toc;
    FrameRate = 1/elapsedTime;
end

close all;
close(writerObj);