%PME 525600
%Special Topics in Mobile Robots and Self-Driving Cars
%HW#1
%105033239_Jim Lee

%*************Code Begin***************

%*********Build the system spec*********
l = 7.5 / 100; 
r = 3 / 100;

%****Define Time****
dt = 0.01;
t = 0:dt:5;

%****Initial Condition****
initPos = [0, 0, 0];
nowPos = initPos;
goalPos = [4, 4, -90 / 180 * pi];
wr_list = (length(t));
wl_list = (length(t));
v_list = (length(t));
w_list = (length(t));
x_list = (length(t));
y_list = (length(t));
theta_list = (length(t));
rho_list = (length(t));
alpha_list = (length(t));
beta_list = (length(t));

for i = 1:length(t)
    %Check whether it's a goal state or not
    rho = sqrt((goalPos(1)-nowPos(1))^2 + (goalPos(2)-nowPos(2))^2);
    if(rho < 0.01)
        fprintf("-----Reach Goal State-----\n")
        v = 0;
        w = 0;
    else
        fprintf("-----%f seconds have passed-----\n", i * dt);
        % Calculate rho, alpha, beta at now position
        rho_list(i) = sqrt((goalPos(1)-nowPos(1))^2 + (goalPos(2)-nowPos(2))^2);
        alpha_list(i) = -nowPos(3) + atan((goalPos(2)-nowPos(2)) / (goalPos(1)-nowPos(1)));
        beta_list(i) = -nowPos(3) - alpha_list(i) + goalPos(3);
        % Now, implement the Control Law
        K_rho = 3;
        K_alpha = 8;
        K_beta = -2.5;
        v = K_rho * rho_list(i);
        w = K_alpha * alpha_list(i) + K_beta * beta_list(i);
    end
    nowPos(1) = nowPos(1) + v * cos(nowPos(3)) * dt;
    nowPos(2) = nowPos(2) + v * sin(nowPos(3)) * dt;
    nowPos(3) = wrapToPi(nowPos(3) + w * dt);
    fprintf("Now Position at \nX: %f\nY: %f\nTheta: %f\n", nowPos(1), nowPos(2), nowPos(3) * 180 / pi);
    x_list(i) = nowPos(1);
    y_list(i) = nowPos(2);
    theta_list(i) = nowPos(3);
    %****To Find Wr and Wl****
    %****Use Cramer's Rule****
    delta = 0.5 * r * 1/(2 * l) * (-r) - (0.5 * r * 1 / (2 * l) * r);
    delta_wr = v * 1/(2 * l) * (-r) - (w * 0.5 * r);
    delta_wl = 0.5 * r * w - (v * 1 / (2 * l) * r);
    wr_list(i) = delta_wr / delta;
    wl_list(i) = delta_wl / delta;
    
    plot(nowPos(1), nowPos(2),'-s','MarkerSize',10,'MarkerFaceColor','b');
    xlabel('x(m)')
    ylabel('y(m)');
    hold on
    grid on
    plot(x_list, y_list);
    hold off
    axis([0 5 0 5])
    drawnow
    
end

% Plot Time Response of the 2 angular velocity
figure(2)
subplot(2,1,1);
plot(t, wr_list);
xlabel('t(s)')
ylabel('w_r(rad/s)');
title("The Time Response of w_r")
subplot(2,1,2);
plot(t, wl_list);
xlabel('t(s)')
ylabel('w_l(rad/s)');
title("The Time Response of w_l")

% Plot Time Response of the x, y, theta compenents
figure(3)
subplot(3,1,1);
plot(t, x_list);
xlabel('t(s)')
ylabel('x(m)');
title("The Time Response of x_c")
subplot(3,1,2);
plot(t, y_list);
xlabel('t(s)')
ylabel('y(m)');
title("The Time Response of y_c")
subplot(3,1,3);
plot(t, theta_list * 180 / pi);
xlabel('t(s)')
ylabel('\theta')
title("The Time Response of theta")