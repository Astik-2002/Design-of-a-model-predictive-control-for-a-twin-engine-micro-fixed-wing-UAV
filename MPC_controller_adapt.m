%mpc control of longitudinal and lateral dynamics of a fixed wing UAV
%considering longitudinal control
%A_long = [X_u   X_w    X_q    X_theta;
%           Z_u   Z_w    Z_q    Z_theta;
%           m_u   m_w    m_q    m_theta;
%           0     0      1        0];
% B_long = [X_e X_t; 
%           Z_e Z_t; 
%           M_e M_t;
%           0   0];
%Value of corresponding stability and control derivatives is obtained using
%xflr5 software.
A = [-0.0136411            0.240871                   0               -9.81;
     -0.696212            -8.97341             27.0777                   0;
      0.308711            -16.6181            -9.98993                   0;
      0                   0                   1                   0];
B = [ 0.9058396 2;
     -29.71347 0;
     -448.2916 0;
      0 0];
%since velocity pertubrations in x and z directions cannot be measured directly, the
%output y looks like [q theta]. Hence
C = [0 1 0 0;0 0 0 1];
D = [0 0;0 0]; %no dependancy of sensor output on inputs
plant = ss(A,B,C,D);
x0 = zeros(4,1);
MV = struct('Min',{-10,-25},'Max',{15,25},'ScaleFactor',{50,50});
OV = struct('ScaleFactor',{60,60});
Weights = struct('MV',[0 0],'MVRate',[0.1 0.1],'OV',[10 200]);
%theta assigned higher weight than pitch rate.
mpcobj = mpc(plant,0.05,10,2,Weights,MV,OV);

%Lateral controller
%since the aircraft has 2 engines, it is capable of yaw control by the
%differential thrust, hence the control matrix B needs to be modified.
%incorporation of diff. thrust can be seen in this paper https://doi.org/10.1155/2018/8654031
A_lat =  [0.0681722           0.0200448            -28.2138                9.81;
          0.169101            -12.8869             1.97109                   0;
          0.201071            -1.39567          -0.0607525                   0;
          0                   1                   0                   0];
B_lat_og = [0.9485224 -0.637;
           8.038241 557.531;
          -16.67931 33.4229;
           0 0];
                 
%dT*ye = I_zz*(r') : relation b/w differential torque and yaw rate
%effects of dT on v are ignored as the time of operation of differtial
%thrust is assumed to be short
B_lat = [0.9485224 -0.637 0;
           8.038241 557.531 0;
          -16.67931 33.4229 3.24;
           0 0 0];
c_lat = [0 1 0 0;0 0 1 0];
d_lat = [0 0;0 0];
