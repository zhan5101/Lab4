%[text] Step 1: Create Trapezoidal Velocity Trajectories
clear all;
close all;
clc;
%[text] (1a) Calculate arc length of Lissajous Curve.
% TODO: replace T, xd, yd with your our lissajous curve
T = 2*pi;
t = linspace(0,T,1000);
A = 0.15;
B = 0.15;        
a = 3;
b = 2;        

xd = A .* sin(a .* t);
yd = B .* sin(b .* t);

d = 0;
for i=1:length(t)-1
    d = d + sqrt( (xd(i+1)-xd(i))^2 + (yd(i+1)-yd(i))^2 );
end
%[text] Determine the average speed, $c$, of end effector over `tfinal` seconds.
% TODO: replace tfinal with your code
tfinal = 15;

% calculate average speed
c = d/tfinal;
%[text] Use forward-euler method to numerically approximate $\\alpha(t)$
% normalized trapezoidal curve
g = @(t, T, ta) (T/(T-ta))*((t < ta) .* (t / ta) + (t >= ta & t <= (T - ta)) .* 1 + (t > (T - ta)) .* ((T - t) / ta));

dt = 0.002;
t = 0:dt:tfinal;
alpha = zeros(size(t));
for i=1:length(t)-1
    % xdot = ...
    % ydot = ...
    % equation (7) in the lab assignment
    % alpha(i+1) = alpha(i) + ...
end

plot(t,alpha,'LineWidth',2); title('Plot of \alpha(t)'); %[output:8859e66a]
grid on; xlabel('time (s)'); ylabel('\alpha(t)'); %[output:8859e66a]
yline(T,'k--','LineWidth',2);  %[output:8859e66a]
legend('\alpha','T (period)','Location','southeast') %[output:8859e66a]
%[text] (1c) Combine all trajectories together.
% TODO: replace with your own lissajous curve
% x = 0.16*sin(alpha);
% y = 0.08*sin(2*alpha);
%[text] Plot the speed of the trajectory as function of time.
v = sqrt( (diff(x)/dt).^2 + (diff(y)/dt).^2 ); %[output:2b3ed4fd]
plot(dt:dt:tfinal,v,'LineWidth',3); hold on;
yline(c,'k--'); 
ylim([0 0.3])
yline(0.25,'r--')
hold off;
title('Trajectory Velocity')
grid on;
xlabel('time (s)')
ylabel('velocity (m/s)')
legend('velocity', 'average velocity', 'velocity limit','Location','south')
%%
%[text] ## Step 2: Forward Kinematics
%[text] (2c) Calculate T0
% these values were obtained from the URDF directly
L1 = 0.2435;
L2 = 0.2132;
W1 = 0.1311;
W2 = 0.0921;
H1 = 0.1519;
H2 = 0.0854;

% home position of end effector
M = [-1 0 0 L1+L2;
    0 0 1 W1+W2;
    0 1 0 H1-H2;
    0 0 0 1];

% screw axes
S1 = [0 0 1 0 0 0]';
S2 = [0 1 0 -H1 0 0]';
S3 = [0 1 0 -H1 0 L1]';
S4 = [0 1 0 -H1 0 L1+L2]';
S5 = [0 0 -1 -W1 L1+L2 0]';
S6 = [0 1 0 H2-H1 0 L1+L2]';
S = [S1 S2 S3 S4 S5 S6];

% body screw axes
B1 = ECE569_Adjoint(M)\S1;
B2 = ECE569_Adjoint(M)\S2;
B3 = ECE569_Adjoint(M)\S3;
B4 = ECE569_Adjoint(M)\S4;
B5 = ECE569_Adjoint(M)\S5;
B6 = ECE569_Adjoint(M)\S6;
B = [B1 B2 B3 B4 B5 B6];

% joint angles
theta0 = [-1.6800   -1.4018   -1.8127   -2.9937   -0.8857   -0.0696]';

% calculate the 4x4 matrix representing the transition
% from end effector frame {b} to the base frame {s} at t=0: Tsb(0)

% TODO: implement ECE569_FKinSpace and ECE569_FKinBody
T0_space = ECE569_FKinSpace(M,S,theta0)
T0_body = ECE569_FKinBody(M,B,theta0)
T0_space-T0_body
T0 = T0_body;
%[text] Calculate Tsd at every time step.
% Calculate Tsd(t) for t=0 to t=tfinal
% Tsd(t) = T0 * Td(t)
N = length(t);
Tsd = zeros(4,4,N);
for i=1:N
    Td = [eye(3), [x(i); y(i); 0]; 0 0 0 1];   % Rd = I3, pd = [x(t); y(t); 0]
    Tsd(:,:,i) = T0 * Td;
end
%%
%[text] (2d) Plot (x,y,z) in the s frame
xs = Tsd(1,4,:);
ys = Tsd(2,4,:);
zs = Tsd(3,4,:);
plot3(xs(:), ys(:), zs(:), 'LineWidth', 1)
title('Trajectory \{s\} frame')
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
hold on
plot3(xs(1),ys(1),zs(1),'go','MarkerSize',10,'LineWidth',2)
plot3(xs(end),ys(end),zs(end),'rx','MarkerSize',10,'LineWidth',2)
legend('Trajectory', 'Start', 'End')
grid on
hold off
%%
%[text] ## Step 3: Inverse Kinematics
initialguess = theta0;
Td = T0;

% you need to implement IKinBody
[thetaSol, success] = ECE569_IKinBody(B,M,Td,theta0,1e-6,1e-6);
if (~success)
    close(f);
    error('Error. \nCould not perform IK at index %d.',1)
end
%%
%[text] (3c) Perform IK at each time step
thetaAll = zeros(6,N);
thetaAll(:,1) = theta0;

% you can comment out the waitbar functions if they aren't working
% (sometimes they don't work with .mlx files)
% If the code gets stuck here, you will need to restart MATLAB
f = waitbar(0,['Inverse Kinematics (1/',num2str(N),') complete.']);

for i=2:N
    % TODO: use previous solution as current guess
    % initialguess = ...
    initialguess = thetaAll(:, i-1);

    % TODO: calculate thetaSol for Tsd(:,:,i) with initial guess
    % [thetaSol, success] = ...
    [thetaSol, success] = ECE569_IKinBody(B, M, Tsd(:,:,i), initialguess, 1e-6, 1e-6);

    if (~success)
        close(f);
        error('Error. \nCould not perform IK at index %d.',i)
    end
    thetaAll(:,i) = thetaSol;
    waitbar(i/N,f,['Inverse Kinematics (',num2str(i),'/',num2str(N),') complete.']);
end

close(f);
%%
%[text] (3c) Verify that the joint angles don't change very much
dj = diff(thetaAll');
plot(t(1:end-1), dj)
title('First Order Difference in Joint Angles')
legend('J1','J2','J3','J4','J5','J6','Location','northeastoutside')
grid on
xlabel('time (s)')
ylabel('first order difference')
%%
%[text] (3d) Verify that the joints we found actually trace out our trajectory (forward kinematics)
actualTsd = zeros(4,4,N);
for i=1:N
    % TODO: use forward kinematics to calculate Tsd from our thetaAll
    actualTsd(:,:,i) = ECE569_FKinBody(M, B, thetaAll(:, i));
end

xs = actualTsd(1,4,:);
ys = actualTsd(2,4,:);
zs = actualTsd(3,4,:);
plot3(xs(:), ys(:), zs(:), 'LineWidth', 1)
title('Verified Trajectory \{s\} frame')
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
hold on
plot3(xs(1),ys(1),zs(1),'go','MarkerSize',10,'LineWidth',2)
plot3(xs(end),ys(end),zs(end),'rx','MarkerSize',10,'LineWidth',2)
legend('Trajectory', 'Start', 'End')
grid on
hold off
%%
%[text] (3e) Verify that the end effector does not enter a kinematic singularity, by plotting the determinant of your body jacobian
body_dets = zeros(N,1);
for i=1:N
    body_dets(i) = det(ECE569_JacobianBody(B, thetaAll(:, i)));
end
plot(t, body_dets)
title('Manipulability')
grid on
xlabel('time (s)')
ylabel('det of J_B')
%%
%[text] (3f) Save to CSV File
% you can play with turning the LEDs on and off
led = ones(N,1);

% save to the CSV file
data = [t' thetaAll' led];

% TODO: change the csv filename to your purdue ID
writematrix(data, 'he711.csv')

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":19.8}
%---
%[output:8859e66a]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAO4AAACQCAYAAAAPxleTAAAAAXNSR0IArs4c6QAAFwlJREFUeF7tXWlsVkX3P2gRDMvfSI0UEEXeQlyrAmldgkHCPxpDFBMtoiBQa11YIhRKUSti2FrAgPqholQwCn5x\/0Be1KQuYPMCBqOiCNoiFKRUlE0UQt+ceTNP5hnuMnfm3vvc5dykoeWemTnb754ze6eOjo4OoIc0QBqIlQY6EXBjZS9iljTANEDAJUcgDcRQAwTcGBqNWCYNEHDJB0gDMdQAAVfDaEuWLIH6+nrbkm+99RaUlJRk3nP60aNHw+LFi+H888\/33OpXX30FWM9rr70GF154oefyqgVk2RYtWgSlpaWqxZXpfv\/9dygrK2P0okyynHZ0yg0llJCAq2FYN+BilaLDmwL37bffhurqaigqKgoUuAiacePGZWlE\/ghpqMuyCJepoqICqqqqGI2dnFx\/QX1E\/JIpzHoIuBraVgGuCLK4AbdPnz7Q0NAAhYWFGtpxL\/LXX3\/BnDlz4MMPPwTxw2AHXP5BMclY3LmKFwUBV8NeTkDkzic6vxO9\/BEQnVN0cJFNHQd2agfrtvoY6UT4n376CSZNmgStra2MZYyo5eXlLC3evn07IO\/33HMPo+H1Y9eBA9lKTtQDLx9UBqDhBjktQsDVUL8fwOV9N3Rm+eGg79evn6NDq\/SVVdrByOoHcK1SbVk2BF5jYyMbI+AfIKRxAq74XkytNUyXmCIEXA1T+pEq8zrEyCyCTIyqJn1cL+1w4Omkyna8i7pCmWpqamD+\/PksTZZB6CSnaXdDw8yRLkLA1TCPCnDFlE52OjH1kwdcrMCjC1wRTCrtmADXqouAqhVTZ+Rh1KhRmbRX5slJTl0daJg3FkUIuBpmcgKuVbSSgbt3795MP1Dus4mOzt\/pOq1VXVxcq3cmwOUyylGUt3PRRRexEfH29nZb2Z3kNOFNw8SRL0LA1TCR17Qt6cAVB9HsIjtP\/Z0+WgRcdWck4KrrKkNpCtykpcoicOWI6+WjpQJcnZFuDRNHvggBV8NEpsDF0WAvg0a6qbI4zaMyCGaSjlrJI44y84jr9NGiPq66MxJw1XXlW8RF4DpN08hRRZ5m8TKPq9OOzqiy21SQ1ZytHJ2d5PT6sdQwa6yKEHA1zOXViXQXYHDW5IUYOumi2wIMbMsk4orlOd\/Y38VHXq7JI6v8AbKTU1ygQcse\/6ddAq4GcKmImQb4B0L1AyRmDbRyioBr5n1UWlsDdmuV7Sqktcpna4Yirrb7UUETDVjtDrKrj3YHEXBNfI3K+qgB1X22qnQ+shaLqijixsJMxCRpIFsDBFzyCNJADDUQOeDiFIHJg7tOdB\/TtrFdk\/axvCkPJu2btk3ym9tf1XcjBVxcxzp27FhV3i3pli5dql2+srJSuywvaNI+1mHKg0n7pm0j\/ybtJ0l+3EuNP0E9kQEugnbWrFnwwQcfGMl68cUXa5f\/7bfftMvygibtYx2mPJi0b9o28m\/SfpLkLy4uhrq6usDAGxng8rk6PMGwa9eu2gAy+eKHGXFaWlrghx9+gJEjR0JeXl5GXlMeoii\/naxWRk6C\/E1NTbBixYqs87S0HdqmYOSAm5aVMSdPnoT9+\/dDQUGB0YfKb4cIor40yYr640EoSF8m4AbhqQp1psmZ0yQrAVfB+eNMkiZnTpOsBNw4o1KB9zQ5c5pkJeAqOH+cSdLkzGmSlYAbZ1Qq8J4mZ06TrARcBeePM0manDlNsiYKuHbHp4inGYQxhB4loKfJmdMka6KAi2frzp49G2pra20vkiLgRumz4i8vBFx\/9Ym1hTKPq3K3KwHXf+NGpUYCrv+WCAW4eNpBc3Nz5h5UKzEIuP4bNyo1EnD9t0TgwLW7KtLuxPsgl4n5rz79GtPkzGmSNTF9XD4w1b9\/f1i8eDHgUZv8PpmpU6dCaWkp834ecadPnw533XUX9O7dWx8VMSiJznzgwAG2m0bluswYiGTLYppkRZtu3bqV7XQLMggFHnHtrInp8\/r169lFULgjSDwM+6GHHoIJEybE2Vddef\/777+hra0N8DKsLl26uNLHmSBNsq5duxbWrFnDzJVI4CJQcQtXQ0MDG2nmwMU9jEOGDEl8xMUuBO5\/xczCZBtjHACdJlkx4r7\/\/vvJ3dZnB9wgv1JRcvI09fvSJGti+ri8P4sbvEtKSjLYwVR506ZNmX4vjSpH6bPiLy8EXH\/1Gdo8Lh5ojcCU+7NidCXg+m\/cqNRIwPXfEqENTsmXTskpMQHXf+NGpUYCrv+WCA24bqwTcN00FN\/3BFz\/bUfA9V+nSjWmyZnTJGtiBqeUvDikA7ZUeQmDLk3OnCZZCbhhoCeHbaTJmdMka+SAi5Po+IPL84JYokd93Bx+RQJumoDrv4Jt+7hHjx6FjRs3wnvvvQd4wPOpU6cyrXfu3BnwpHZcZzx8+HDo0aOHMWcEXGMVRrYCAq7\/pjkLuMePH2fLEF988UXW2nXXXQc33HADW088cOBA2L17N+DGAQTzt99+y6JveXk54PpiEwATcP03blRqJOD6b4kMcDs6OuCzzz6D+fPnM4BOnjyZAfa8886zbRVT5y1btsDrr78OP\/74IyxYsIBF4E6dOnnmNNfAxbuL8Ces5\/Tp03Do0CG44IILEr9WOWmyul3oFYYvZ4CLURSjLN6WN2jQIE\/gQ9Dv3LmT7fbBrXoYnb0+YQhrxxO\/cAyzCHpIA24acLvQKwxfpnlcYSoKdyb17dvXzW70PsUaULnQK2fAxf2T33zzDQwYMADy8\/MtzYQXVn3yySdw9913Q\/fu3Y1NGYawdkzmsm1jxVEFoWpAxVdUaEyZtoy4\/NSKqqqqrB09YmN48\/nq1aszGwdMGQlDWAKuqZWovIqfqtCYajIDXIyyeKfn5s2b4Z9\/\/oFdu3axS3l79ux5Vhv8\/YgRI9gN5BRxTc1A5eOiARVQqtCYypsVcXfs2MFGlY8cOeIIXBwJxVMqxowZ41ufMAxhKeKauguVV\/FTFRpTTVqmyocPH4aamho2JXT99debtqFUPgxhCbhKpiAiBw2o+KkKjamSaVQ5hRscTJxGPG63T58+bLHOtm3bsk4zMak\/6mVVQKlCYypn1jzuwoULYfz48XDttdd6nsfFUWicx8VjKeM2jxuGok0NFYXyHLQIWBy4xEHM5cuXw759+6CiosJ2IDMKvPvFg4qvqNCY8nPWyql58+bB5ZdfDhMnToShQ4c6big4duwY+9LikTQHDx4ELBvHlVNhKNrUUFEoj3rCk0z4EUTIk3wsURT4DJIHFV9RoTHl0Xat8qpVq9huoMGDB8OwYcOy1iG3t7ez0edffvkFevXqxdYqP\/DAA9CtWzdtfsIQVqeP+8WuP2DK+h3acsWt4Etjr4Bb\/nWBJdsIUnww2vIHD\/3Dhx9sHzd5vfKr4qcqNF7blelt+7gIWgQnGgYZwd1C\/EGw4qjyfffdBzfeeKMv2\/zCEFYHuOv+cwCeWJce4H7w+PWWwOVp8k033ZQBKf+\/+++\/PxVpMvqPip+q0AQGXLniM2fOwIkTJ1hU1dlE4MZoGMIScN2sAOAFuHZH77q3El8KFT9VoTHVgO3KqZkzZ8LcuXNt77M1bVguH4awBFx3q9kBF0uKZ2Hj3ytXrgRcQSefme3eSnwpVPxUhcZUA45LHrdv387qxxFDsV9j2qhV+TCE1QHunt9PBiFuZOvsf2FXW97kqaCXX36ZLXulVDlbZWH4suM8rnwWsmzRoqIiWqscWQgSY0FoQAWUKjSmvNkCl\/dfWltbbdsg4Jqqn8rHTQMqoFShMZXbErg8JcLK+Z22vCHO1OjRo896Z8JMGMLqpMomMlHZ5GlAxU9VaEw1o72tz2pOz4SZMIQl4JpYiMqiBlT8VIXGVJuOEZcvbbMbTJJX0ZgwE4awBFwTC1HZyANXZBB\/ly\/o4qn0nj17aHCK\/DlVGlAJMCo0pkpzHFXmDNg14uc0URjCUsQ1dRcqr+KnKjSmmlTa1mc1LbRo0SJf16eGISwB19RdqLyKn6rQmGpSCbimjaiUD0NYAq6KJYjGSQMqfqpCY6plAq7iSKGpoql8MjSgAkoVGlNtEHAJuK4+5LSCjp+CUVhYaFkPLuTBAxqWLVumdcCCG3Pi+mmvl9HJp5mqbFFUAaUKjZtcbu8JuARcNx\/Jeo9OuW7dOqXFNzj7gNfS4L1SdsD21LjPxDJwVfhVAaUKjakoBFwCricf8gJcpG1sbMxsUMGI1tzczNqrr69n\/8pTjfJMhvgey7e1tbGD+HEDDL5raWnJOu9KLG+VDWAd1dXVrG1c\/YdTmuL54W7yqYBShcaT0i2ICbgKwEUDmzy49U33MW0b2zVpX+bbzbE5vdUmew4aPiMh7+fFusVFPfh+9uzZUFtbyyI2lsf7rfCAOh7BxVQZwVxZWZl5j\/WJfyMtnovGj97hXQDx44BR2GlLqwooVWh0\/YGXI+AScD35kCpwrQBg1R\/lS2enTZsGc+bMOWuLII\/SGBWtyvP\/w+OE8Uxw8YQOFEyuX3xvdWOH1UkfooJUQKlC40npFHGt1eWmaNOoZxLxTNvOVcS1GpQSQcgtwYGHkXXKlCksBZYfvtDHCbi8vHxtDm8Tz0UrKyvLSovtjt5xWofv5ivIuwoNAddUAwqKNgVPGoErp71oJhXgOt1XFRZwrfjkbqYCShUaU7elVFkBuKZKTlJ51VTZLuLicb7iVlGnVFbWWxipspheW536ogJKFRpTnyDgEnA9+ZAqcO36uDiiyweD5MEneTBJ7m86ARc\/Bm6DU3IWYDU4RX1cT+4QTr\/AjqUwvpAe1RFZclXg2o0qb9iwgcmG00T4yNNB4nQNvhfXxLsBFxdgeJkOmjFjBptaEtNzGlX26Hq5BE8u2\/aopliRyyA3WeUUluDy3LPcroqvqNCYykOpMqXKpj5kW15eiRR14NLKKQ1XCOMrRamyhmEMi4iDVBs3boz0rX60VlnD2ARcDaVRkdA1oOKnKjSmjFOqLKTK06dPh+LiYlOdUvkEawCvFMWrZOVBNVFkAm5IDrB3715mjKamppBapGbirAH8uNfV1UG\/fv0sxSDghmhdBC\/+hPWcPHkS\/vjjD8jPz4e8vLywms1JO0mTFQFrB1pUcKKAK86voXB227mcUpCceF1AjaIz79+\/HwoKCqBrV\/v7egJqPtRq0yRrooArr4iR\/w5L2FC91aWxNDlzmmQNy5cDH5ziK2jkw9XlHRhhpBcE3NxogIDrv94DB67Vnkf+VRI3TRNw\/TduVGok4PpvicCBKy8k5yJYpc\/jxo1zHGb3X\/zc1ZgmZ06TrIlJlb0Ad+zD0+DI\/y\/JHZqoZdKATxro+e8qWP\/qSigpKfGpxuxqIhNxcSpm5rNLYOP\/3ROIoFQpaSBMDYz68x1Y9lyV47SRCT+RAS4KEfZcqoniqCxpwEkDbnO9ptoLHLiqg1OmglB50kCaNBA4cFWng9KkdJKVNGCqgcCBazXKZrUAw1QQKk8aSJMGQgGuCF6uXHFpI4\/K\/DREPFVRPFAsSQaRj2ZB2YqKiny7IDwqunK6M8ht+WtUZFDlw05W3k2Uj5z144ra0IBrpwQ5lbZLrVWVGHU6pzN7o867Kn\/cYZGe3xrAy6osf1VtJwp0TrLaTYX6wXfOgWuVNgcpsB9K063D7QRB3XqjVE6MpnImkbTxDidZeZYprg700045B67VOURJdXC3EwT9NGwu6uKOjKkgPuI9Pfh3kmYY3GRFeZ0OVje1T86Ba5U6JjVdlvt2Se3fcqeVgau6is7UqcMuL18mhu3L4zacJz\/6t1gXATdEK\/OBKXFgDj9cCGi5LxgiW4E0ZeXMaQIuzy769++fGWjltxNOnToVSktLjfROwDVSn3lhbuCxY8caG9OcG\/9qSDtw7TRppRcdrRNwdbTmY5mkdgsIuNZO4tcahpwDN02DU1amTBNwkzQ4JdrSSxRNDHDTNB2E\/dnW1tasxSV2zuxjUM9JVU4DNm6noeSEYYNG7bKLSZMmwdKlS7O29vl1m0POIy6POKg3XC2FD95MLhvXQK+RKSoPTsiy46VVSXnsopB80olfESiXerOTVR549POUl5wD12roPMlLHjl4MfLik1RZndLHpC15dJKVX+XJPyx+nWIaCeDm8mtJbZMG4qgBAm4crUY8p14DBNzUuwApII4aIODG0WrEc+o1QMBNvQuQAuKoAQJuHK1GPKdeAwTc1LsAKSCOGiDgRsBqe\/bsgQ0bNsD48eMBF2H4OVHvh3hffPEF21u6cOFC6NGjh2uVzc3NsGDBApg3bx707dvXlZ4IvGuAgOtdZ76XkJdCRgm4bW1tUFlZCeXl5XDLLbcoyd7R0QFr166FlpYWmDt3buLv\/1VSis9EBFyfFapTndUaZp16giiDANy6davnw\/v27dsHuO\/0qaeegiFDhgTBWqrrJODm0PxWpyTgCQmXXnopiBeg8fW8y5cvZyn1m2++yS7DfvTRR2HChAnw3XffwfPPP8\/+xbtqqqur4ZprrslIdvz4cRYBV69eDe3t7XDVVVfBI488AnfccYdjNDxw4ABUVFQw2jvvvDNTH4ISPzaNjY3spIerr76agfTWW2+Fc845h9Fh1K2rq4NDhw4x3rp06ZJDTSevaQJuDm16+vRpBrZXX32VOTimo4WFhYDAkIGL7\/Lz81n0uv3226GpqQnWrFnDft+1axcr26lTJ3jllVegc+fOUF9fD7179wae6mI\/+uGHH2Z32WzevBkaGhoY8BFweXl5llr49NNPWb8WaS+55BJGg\/VNmTKFARR57NatG3z00UfsZ9WqVQy8\/MEPzsyZM5l8V1xxRQ41nbymCbgRsKlbH5f3eRFoM2bMYEA7duwY+\/37779nx94MHjyYSYJgQzqMysOGDWORFgeWEMgIWv4g3TPPPGMLKvyoYKTEbYe4awsBig8HI4J50KBB7P+OHDnC+sFXXnklTJs2LRN1f\/31V8CtbU888QSMGTMmAppODgsE3AjYUhW4CEI+QMTT7DNnzmQBa+fOnQwsy5YtY+kybpE8ceIES6nPPffcrHQX94oi4KzOP+IfhoEDB0JVVVWm3I4dO2DixIkwYsQIlkZjWs\/TY1mVCGiMzpiai3VEQOWxZ4GAGwETqgLX6vYHZF+89YFvG0RQYkQsKysD+SR9UWSMhpjOyg\/f4D9y5EgGPv5gJMYUvba2Fk6dOgW9evWCUaNGwb333stuZBBBnOT9xrl2GwJuri0AwAZ6xJMxrDabi31eZNkOFFbAxQErrxHP7RC7o0ePwpdffgnvvvsuS5\/xb+wvi31mAm5wzkXADU63yjUHBVwc7cV+MD44It29e3dlnuwirlUFCNAVK1bAxx9\/nDWQRcBVVrdnQgKuZ5X5XyAo4GKkxX4xTsu89NJLWSO+OJWDURhHjW+77bazhOJ93Msuu4xNL+GINT7vvPMOS5URqPgOH5z6wammdevWZQGX+rj++wqvkYAbnG6Va0ZQvfHGGzBr1iwoLi62nA7SSZURuAcPHmQjvV9\/\/TWbjx06dChs27YNsL88fPhweO655ywjMYIR55Rx+aIYrXfv3s3q6dmzJxvUKigogC1btrAPxOTJk9kIMp9e4qPKTz75ZNY8sLJiiNBWAwTcCDjHzz\/\/DE8\/\/TTrK6Lj33zzzWfN4+oCF8U7fPgwi4h4JQguwMD1w1gfjjTzaR4rNeCUEa45xukmHl2RDsH7wgsvwOeff876tgMGDIDHHnuMnZ8lLrTA8jU1NSwK4\/w0Pf5pgIDrny4TVxNfbPHggw8yUHp5cPQZuwB\/\/vknrZzyojhFWgKuoqLSSoYp8KZNm9j5wF4GtzDFfvzxxxloaa2y\/95DwPVfp4mq0WR3EC4GwVSZ1in77xIEXP91mrgadfbjPvvss2zgS+wbJ04xORTov99CHblKOX42AAAAAElFTkSuQmCC","height":96,"width":159}}
%---
%[output:2b3ed4fd]
%   data: {"dataType":"error","outputData":{"errorType":"runtime","text":"Unrecognized function or variable 'x'."}}
%---
