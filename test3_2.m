clc, clear all, close all

%%%%%%%%%%% case 1 %%%%%%%%%%%%%%%%%%%
% Set up the initial positions of the red ship and obstacle
init_position = [0, 2];
goal_position = [0, -2];

obstacle_position = [-1.2, 0];


% Set up the velocity vectors for the red ship and obstacle
V0 = 0.02;
red_ship_velocity_abs = V0;
obstacle_velocity = [0.014 , 0];


%initial position of red ship
red_ship_position = init_position;

%calculate initial heading angle
heading0 = atan2(goal_position(2) - init_position(2), goal_position(1) - init_position(1));
heading = heading0;
heading_p = heading;

% % %%%%%%%%%%%%% case 2 %%%%%%%%%%%%%%%%%%%%%%%
% % % Set up the initial positions of the red ship and obstacle
% % init_position = [0, -1.5];
% % goal_position = [0, 2];
% % 
% % obstacle_position = [-1, .5];
% % 
% % 
% % % Set up the velocity vectors for the red ship and obstacle
% % V0 = 0.02;
% % red_ship_velocity_abs = V0;
% % obstacle_velocity = [0.014 , 0];
% % 
% % 
% % %initial position of red ship
% % red_ship_position = init_position;
% % 
% % %calculate initial heading angle
% % heading0 = atan2(goal_position(2) - init_position(2), goal_position(1) - init_position(1));
% % heading = heading0;


% % %%%%%%%%%%%%% case 3 %%%%%%%%%%%%%%%%%%%%%%%
% % % Set up the initial positions of the red ship and obstacle
% % init_position = [-1.5, 1.5];
% % goal_position = [1.5, -1.5];
% % 
% % obstacle_position = [-0.85, -0.95];
% % 
% % 
% % % Set up the velocity vectors for the red ship and obstacle
% % V0 = 0.02;
% % red_ship_velocity_abs = V0;
% % obstacle_velocity = [0.011 , 0.011];


%initial position of red ship
red_ship_position = init_position;

%calculate initial heading angle
%angle between x axis and line that passes goal poisition and init
%position.
heading0 = atan2(goal_position(2) - init_position(2), goal_position(1) - init_position(1));
heading = heading0;

% Set up the time step and the duration of the simulation
dt = 1;

% Set up the path array to store the positions of the red ship during movement
path = [red_ship_position(1), red_ship_position(2)];

% Set up the path array to store the positions of the obstacle during movement
path2 = [obstacle_position(1), obstacle_position(2)];

%distance ship and obstacle
%Euclidean distance between two points
dist =  (norm(red_ship_position- obstacle_position)); 

cir_angle = 0;    %angle of circle path for red ship
cir_flag = 0;     %Status on whether the reship is on a circular path
obstacle_pass = 0;%Status on whether the obstacle passes or not
reft_pass = 0 ;   %status on whether the red ship move to left
frames = [];
dcpa_list = []
% Loop over until red ship reaches to goal point
while norm(red_ship_position - goal_position)>0.1
    
    % calculate the velocity of redship
    red_ship_velocity = red_ship_velocity_abs *[cos(heading), sin(heading)];
    % Update the positions of the red ship and obstacle based on their velocities
    red_ship_position(1) = red_ship_position(1) + red_ship_velocity(1) * dt;
    red_ship_position(2) = red_ship_position(2) + red_ship_velocity(2) * dt;  

    obstacle_position = obstacle_position + obstacle_velocity .* dt;


    % Determine the distance between the red ship and the obstacle
    %Euclidean distance between two points
    distance = (norm(red_ship_position- obstacle_position));   
    % redship's relative velocity to obstacle
    velocity_r = red_ship_velocity - obstacle_velocity;
    % redship's relative position to obstacle
    position_r = red_ship_position - obstacle_position;
    % calculate the dcpa from the relative velocity and position of red ship to obstacle
    dcpa = abs(position_r(1)*velocity_r(2)-position_r(2)*velocity_r(1))/norm(velocity_r);
    dcpa_list = [dcpa_list dcpa];
    % calculate the DCPA point.
    position_dcpa_r = dcpa*[-velocity_r(2), velocity_r(1)]/norm(velocity_r);
    position_dcpa = obstacle_position + sign(position_dcpa_r*position_r')*position_dcpa_r;
    % get the line with the direction relative velocity
    x_r = red_ship_position(1):(position_dcpa(1)-red_ship_position(1))*0.02:position_dcpa(1);
    y_r = red_ship_position(2):(position_dcpa(2)-red_ship_position(2))*0.02:position_dcpa(2);
    % get the line with vertical to relative velocity
    x_r1 = obstacle_position(1):(position_dcpa(1)-obstacle_position(1))*0.02:position_dcpa(1);
    y_r1 = obstacle_position(2):(position_dcpa(2)-obstacle_position(2))*0.02:position_dcpa(2);

    disp(dcpa);
    %% form path to avoid the obstacle
    if distance < 1.4 && cir_flag ==0 && obstacle_pass ==0
         cir_flag =1;
       
         %determine the obtable on left or right of path 
         %https://math.stackexchange.com/questions/274712/calculate-on-which-side-of-a-straight-line-is-a-given-point-located
         
         d = (obstacle_position(1) - init_position(1))*(goal_position(2)- init_position(2)) -...
             (obstacle_position(2) - init_position(2))*(goal_position(1)- init_position(1));
         
         if d >= 0
             reft_pass = 0;
         else
             reft_pass = 1;
         end
    
         
    end
    
    %Update the heading angle of the red ship to avoid the obstacle
    if cir_flag ==1
        
        if reft_pass == 0
                        
             %calculate the disired the heading angle
             %ship moves harf circle, threshod angle must be pi, but
             %threshold value pi -pi/4.55 is to smooth switch
             %from circlar path to straight path. 
             %this means ship starts to switch from circluar path to
             %straight path at pi - pi/4.55 angle
             
             if cir_angle > pi-pi/4.55
                 
                 d_heading = heading0;                   %disired heading angle of straight path
             else
                 
                 %angle of circlar path
                 d_heading = heading0 + cir_angle -pi/2 ;%disired heading angle of circlar path
             end
             
            
            
            %caculate the heading angle 
            % these command perform the relax of path when switching from
            % line to circular of when switch from circluar to line
            
            if d_heading >heading
                heading = heading +0.03;
            else
                heading = heading -0.03;
            end

        else
             %calculate the disired the heading angle
             %ship moves harf circle, threshod angle must be pi, but
             %threshold value pi -pi/4.55 is to smooth switch
             %from circlar path to straight path. 
             %this means ship starts to switch from circluar path to
             %straight path at pi - pi/4.55 angle
             
             if cir_angle > (pi-pi/4.55)
                 
                 d_heading = heading0;                   %disired heading angle of straight path 
             else
                 %angle of circlar path
                 d_heading = heading0 - cir_angle + pi/2 ;%disired heading angle of straigt path
             end
            
            %caculate the heading angle 
            % these command perform the relax of path when switching from
            % line to circular of when switch from circluar to line
            
            if d_heading >heading
                heading = heading + 0.03;
            else
                heading = heading - 0.03;
            end

        end
        
        %update angle of circle path
        cir_angle = cir_angle + 0.025;
                
        if cir_angle>pi;
            cir_flag = 0;
            obstacle_pass = 1;
        end
        
    else
        red_ship_velocity_abs = V0;
        heading = heading0;   
        
    end
     
    % 
    
    % Store the position of the red ship in the path array
    path = [path; [red_ship_position(1),red_ship_position(2)]];
    
    % Store the position of the obstacle in the path array
    path2 = [path2; [obstacle_position(1),obstacle_position(2)]];

    dist = [dist;distance];
    
    %Sore the heading angle in array
    heading_p = [heading_p, heading]; 
    
    % Plot the positions of the red ship and obstacle
    figure(1)
    plot(path(:,1), path(:,2), 'r--')
    hold on
    
    plot(path2(:,1), path2(:,2), 'b--')
    
    plot(x_r,y_r,'g--')
    plot(x_r1,y_r1,'r-')

    plot(red_ship_position(1), red_ship_position(2), 'ro', 'MarkerSize', 10)
    plot(obstacle_position(1), obstacle_position(2), 'bo', 'MarkerSize', 10)
    xlim([-2.5, 2.5])
    ylim([-2.5, 2.5])
    
    title('Crossing situation-90 degree approach(Red LINE is dcpa)')
    xlabel('X position(NM)')
    ylabel('Y position(NM)')
    
    pause(0.03)
    hold off
        drawnow;
    frame = getframe(gcf);
    frames = [frames, frame];
    
    
end

% Save the frames as a GIF
filename = 'crossing_obs_trail.gif';
for t = 1:length(frames)
    % Convert the current frame to indexed format
    [ind, map] = rgb2ind(frames(t).cdata, 100);%256
    % Write the indexed image to the GIF file
    if t == 1
        % If this is the first frame, create the file
        imwrite(ind, map, filename, 'gif', 'LoopCount', Inf, 'DelayTime', 0.1);
    else
        % If this is not the first frame, append to the file
        imwrite(ind, map, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.1);
    end
end

figure(2)
plot(1:length(dist), dist, 'b')
title('Distance between ship and obstacle')
xlabel('time(s)')
ylabel('distance (NM)')

figure(2)
plot(1:length(dcpa_list), dcpa_list, 'b')
title('DCPA between ship and obstacle')
xlabel('time(s)')
ylabel('distance (NM)')

heading_p = heading_p/pi*180;
figure(3)
plot(1:length(heading_p), heading_p, 'r')
title('Bearing of the vessel')
xlabel('time(s)')
ylabel('heading angle(degree)')


