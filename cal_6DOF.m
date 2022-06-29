function X = cal_6DOF(Moment_cmd, Thrust, state)
global dt
Num = 1;
save_s = 0;
sum_k = 0;
time = 0;
while(Num < 5)
    [X,dX] = UPDATE_DERIV(Moment_cmd,Thrust, state);
    [X,time,save_s,sum_k,Num] = RK4(X,dX,save_s,sum_k,time,Num,dt);
end
X(7:9) = ang_limit(state(7:9));
end