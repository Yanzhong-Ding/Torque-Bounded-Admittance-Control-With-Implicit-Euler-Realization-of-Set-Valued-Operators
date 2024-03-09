close all;
clear;
clc;
tic;
%%%%%%%%%%%%%%%%%%%%%% 运动模型与论文中相同
% m1=1;m2=1.5;
% L1=0.2;L2=0.3;
% Lc1=0.1;Lc2=0.15;
% J1=0.013;J2=0.045;
%%%%%%%%%%%%%%%%%%%%%% 将系统参数扩大
gain=2;  %两个关节总长度:=gain*(L1+L2)=gain*(0.2+0.3)=gain*0.5
% m1=1*gain;m2=1.5*gain;
L1=0.2*gain;L2=0.3*gain; % 0.4, 0.6
Lc1=0.1*gain;Lc2=0.15*gain;
% J1=0.013*(gain^2);J2=0.045*(gain^2);

%  m1=1*gain*4;m2=1.5*gain*4;%%%%%%%%%%%%%%%%%%%%%%%  增大机械臂质量以使输入力矩增大
  m1=1*gain*3;m2=1.5*gain*3;  % m1=6,m2=9
  J1=(m1*(L1^2))/3;J2=(m2*(L2^2))/3;
%   J1=(m1*L1^2)/3;J2=(m2*L2^2)/3;   real J
  
%%%%%%%%%%%%%%%%%%%%%%
rad=360/(2*pi);
q=[97.18075578 -138.5903778]'; q=q/rad;
d_q=[0 0]';d_d_q=[0 0]';
qd=[0 0]';d_qd=[0 0]';d_d_qd=[0 0]';
g=9.8; F=3;%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 饱和力矩阈值
%        F=3;
K1=[1 0;0 1];K2=[1 0;0 1];
tao=[0 0]'; F_ext=[0 0]';
tao_hat=[0 0]';
% mu=0.1; Kz=10000;%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 环境参数设置
mu=0.1; Kz=1.e+3;

% M_da=[1 0;0 1]; D_da=[100 0;0 100]; K_da=2*[1000 0;0 100];   %%参数001

% M_da=[1 0;0 1]; D_da=2*[100 0;0 100]; K_da=5*[100 0;0 100];
M_da=0.5*[1 0;0 1]; D_da=1*[1 0;0 1]; K_da=1*[1 0;0 1];             %%参数001%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% M_da=50*[1 0;0 1]; D_da=100*[1 0;0 1]; K_da=100*[1 0;0 1]; 

D_di=[100 0;0 100]; K_di=4*[100 0;0 100]; 
D_i=[100 0;0 100]; K_i=1.5*[100 0;0 100];
% Xd=[0 0]';
d_Xd=[0 0]';d_d_Xd=[0 0]';
X0=[0;0]; d_X0=[0 0]';
X_initial=[L1*cos(q(1))+L2*cos(q(1)+q(2)) L1*sin(q(1))+L2*sin(q(1)+q(2))]';
X=X_initial; d_X=[0 0]'; 
Xd=X;q0=q;qx=q;%%%%%%%%%%%%%%%%%%% 将proxy初始位置设置为机械臂末端真实位置

T=0.001; h=T;
t=0:T:24;
j=length(t);
data_q=zeros(2,j);data_d_q=zeros(2,j);data_d_d_q=zeros(2,j);  data_q(:,1)=q;
data_qx=zeros(2,j);data_d_qx=zeros(2,j);data_d_d_qx=zeros(2,j);
data_q0=zeros(2,j);data_d_q0=zeros(2,j);data_d_d_q0=zeros(2,j);data_torque_ext=zeros(2,j);
data_tao=zeros(2,j);
data_error_q=zeros(2,j);data_error_d_q=zeros(2,j);
data_F_ext=zeros(2,j);
data_Xd=zeros(2,j); data_X0=zeros(2,j); data_X=zeros(2,j);
data_d_Xd=zeros(2,j); data_d_X=zeros(2,j); data_d_Xd_d_X=zeros(2,j);data_Xd_X=zeros(2,j); sign_2DoFs=zeros(2,j);
data_normal_cone=zeros(2,j); data_normal_cone_flag=zeros(2,j);
%%%%%%%%%  Initialization of Kikuuwe's method
aa=zeros(2,j);  u_x_star=zeros(2,j); u_x=zeros(2,j);  data_qx(:,1)=qx;   %%%%初始化

% D_i=[100 0;0 100]; K_i=1.5*[100 0;0 1000];  %%参数调整
% M=[0.2 0;0 0.2]; K=4*[100 0;0 100]; B=[100 0;0 100]; L=[5 0;0 5];%%最初参数

M=0.3*[1 0;0 1]; K=20*[100 0;0 100]; B=3*[100 0;0 100]; L=[5 0;0 5];   %%稳定性较好

% K=[100 0;0 100]; B=12*[1 0;0 1];%% 这组参数类似滑模算法PD作用
% M=0.2*[1 0;0 1]; K=1*[100 0;0 100]; B=0.12*[100 0;0 100];
% K=1*[100 0;0 100]; B=0.1*[100 0;0 100];
% M=[0.3 0;0 0.3]; K=2*[100 0;0 100]; B=0.2*[100 0;0 100]; L=[5 0;0 5];

K_hat=K+B/T+L*T;  E=[1 0;0 1];  num=[0 0 0 0 0 0 0 0 0 0]';data_test=zeros(3,j); index=0; sigma=zeros(2,j); data_A=zeros(2,2,j); data_tao_hat=zeros(2,j);
Mat1=E+(inv(M_da+D_da*T))*K_da*T^2; Mat2=K_hat+M/(T^2); data_q0_to_X0=zeros(2,j);  data_sets=zeros(5,j);
d_qx=[0 0]';d_d_qx=[0 0]'; u_s=zeros(2,j); d_d_q0=[0 0]'; d_q0=[0 0]';
%%%%%%%%%
%% 滑模控制V1算法相关虚拟变量  F1>Kr
% %     lambda_unit=30; Kr_unit=1;
%       F1=F; lambda_unit=30; Kr_unit=0.2*F1;
%     lambda = [lambda_unit 0 ; 0 lambda_unit];   Kr=[Kr_unit 0 ; 0 Kr_unit];
%     q_circle=zeros(2,j);  q_diamond=zeros(2,j); q_box=zeros(2,j);  q_x_diamond=zeros(2,j);
%     Mat3 = (inv(Mat2))*[F1 0 ; 0 F1];  Mat4 = (inv(Mat2))*Kr;  
%     C11 = Mat3(1,1)-Mat4(1,1); C21 = Mat3(2,2)-Mat4(2,2);
%     C12 = Mat3(1,1)+Mat4(1,1); C22 = Mat3(2,2)+Mat4(2,2);  num1=0; num2=0;
%% 滑模控制V2算法相关虚拟变量  F>F0
% F0=0.95*F;  
F0=0.95*F; 
% lambda_unit=30;
% lambda_unit=100;
% lambda = [lambda_unit 0 ; 0 lambda_unit];
M_x=M_da;D_x=D_da;K_x=K_da;
% M_s=[0.4 0;0 0.4]; D_s=[0 0;0 0]; 
% M_s=0.2*[1 0;0 1];  D_s=10*[1 0;0 1];  
M_s=0.3*[1 0;0 1];  D_s=10*[1 0;0 1]; 
h_x=(inv(M_x+h*D_x))*h;  h_s=(inv(M_s+h*D_s))*h;   h_x_star=(inv(E+h*h_x*K_x));  f_e_hat=[0 0]'; 
% h_1=(inv(M_s))*h^2; lambda_h=(inv(E+h*lambda));
e_q=zeros(2,j);  e_r=zeros(2,j);  v_x=zeros(2,j); S0=[0 0;0 0]; e_v=zeros(2,j); s_0=zeros(2,j); q_hat=zeros(2,j); q_hat(:,1)=q; v_x_line=zeros(2,j);

Lambda1_unit=10; Lambda2_unit=30;
Lambda1=[Lambda1_unit 0;0 Lambda1_unit]; Lambda2=[Lambda2_unit 0;0 Lambda2_unit];
lambda_1=(inv(E+T*Lambda1)); lambda_2=(inv(E+T*Lambda2));


% M_q_hat=0.3*[1 0;0 1]; C_q_hat=1*[1 0;0 1];
% M_q_hat=0.3*[1 0;0 1]; C_q_hat=8*[1 0;0 1];
M_q_hat=0.3*[1 0;0 1]; C_q_hat=10*[1 0;0 1];
% M_q_hat=M_s; C_q_hat=D_s;
h_q=h*(inv(M_q_hat+h*C_q_hat));
%% 饱和补偿相关参数
xi=zeros(2,j);   d_xi=zeros(2,j);          S_at=zeros(2,j);   
delta_tau=zeros(2,j); xi(:,1)=[0.01 0.01]';   
    %% 其他参数设置
%     f_d=[3 0;0 3];
%     M_da=[1 0;0 1]; D_da=[15 0;0 15]; K_da=[100 0;0 100];
    
for i=2:j
% error_q=q-qd;error_d_q=d_q-d_qd;
% data_error_q(:,i)=error_q;
% data_error_d_q(:,i)=error_d_q;
data_q(:,i)=q; data_d_q(:,i)=d_q; data_d_d_q(:,i)=d_d_q;
data_qx(:,i)=qx;  data_d_qx(:,i)=d_qx; data_d_d_qx(:,i)=d_d_qx;
data_q0(:,i)=q0; data_d_q0(:,i)=d_q0;data_d_d_q0(:,i)=d_d_q0;
data_tao(:,i)=tao; 
data_F_ext(:,i)=F_ext;
data_Xd(:,i)=Xd; data_X0(:,i)=X0; data_X(:,i)=X; data_d_X(:,i)=d_X; data_tao_hat(:,i)=tao_hat;
data_d_Xd(:,i)=d_Xd; 

%% 二维机械臂系统模型
theta_1=m1*power(Lc1,2)+m2*(power(L1,2)+power(Lc2,2))+J1+J2;  theta_2=m2*L1*Lc2; 
theta_3=m2*power(Lc2,2)+J2;  theta_4=m1*Lc1+m2*L1;  theta_5=m2*Lc2;
theta=[theta_1 theta_2 theta_3 theta_4 theta_5];
M_q=[theta_1+2*theta_2*cos(q(2)) theta_3+theta_2*cos(q(2));theta_3+theta_2*cos(q(2)) theta_3];
C_q=[-theta_2*sin(q(2))*d_q(2) -theta_2*sin(q(2))*(d_q(2)+d_q(1));theta_2*sin(q(2))*d_q(1) 0];
g_q=[theta_4*g*cos(q(1))+theta_5*g*cos(q(1)+q(2));theta_5*g*cos(q(1)+q(2))];
% data_M_q(:,:,i)=M_q;
% data_C_q(:,:,i)=C_q;

t1=i*T;
% a=0.01; b=0.2; omega=2*pi/5;

if(t1<=17.5)
% if(t1<=13.75)
    a=0.01;b=0.2;omega=1*pi/5;
%     a=0.005;b=0.4;omega=2*pi/5;
    X0=[X_initial(1)+a*t1 X_initial(2)+b*sin(omega*t1)]'; %%期望轨迹在正弦低点停住
    d_X0=[a b*omega*cos(omega*t1)]';
    d_d_X0=[0 -b*omega*omega*sin(omega*t1)]';
else
    X0=[X0(1) X0(2)]';
    d_X0=[0 0]';
    d_d_X0=[0 0]';
end
%%
J=[-L1*sin(q(1))-L2*sin(q(1)+q(2)) -L2*sin(q(1)+q(2));L1*cos(q(1))+L2*cos(q(1)+q(2)) L2*cos(q(1)+q(2))];
T_q_to_X=[L1*cos(q(1))+L2*cos(q(1)+q(2)) L1*sin(q(1))+L2*sin(q(1)+q(2))]';
X=T_q_to_X; 
if (i==1)
    d_X=[0 0]';
else
    d_X=J*((data_q(:,i)-data_q(:,i-1))/T);
end
%% 外部交互力矩设置
baseline=X_initial(2)-0;
if (T_q_to_X(2)>baseline)
    F_ext=[0 0]';
else
    if (t1<=100)
        F_ext=[-mu*SignFn(d_X(1))*Kz*(baseline-T_q_to_X(2)) Kz*(baseline-T_q_to_X(2))]';
    end
end
% if (T_q_to_X(2)>X_initial(2))
%     F_ext=[-mu*sign(d_X(1))*Kz*(X_initial(2)-T_q_to_X(2)) Kz*(X_initial(2)-T_q_to_X(2))]';
% else
%     
%     F_ext=[0 0]';
%     
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% control input
%%  
%% impedance control 001
% tao=J'*(15*(d_X0-d_X)+100*(X0-X));
% tao=[project([-F,F],tao(1)) project([-F,F],tao(2))]';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 常规位置控制算法 算法1     AdmC
% J0=[-L1*sin(q0(1))-L2*sin(q0(1)+q0(2)) -L2*sin(q0(1)+q0(2));L1*cos(q0(1))+L2*cos(q0(1)+q0(2)) L2*cos(q0(1)+q0(2))];
% d_J0=[-L1*cos(q0(1))*d_q0(1)-L2*cos(q0(1)+q0(2))*(d_q0(1)+d_q0(2))  -L2*cos(q0(1)+q0(2))*(d_q0(1)+d_q0(2))
%       -L1*sin(q0(1))*d_q0(1)-L2*sin(q0(1)+q0(2))*(d_q0(1)+d_q0(2))  -L2*sin(q0(1)+q0(2))*(d_q0(1)+d_q0(2))];
% d_d_q0=(inv(J0))*(d_d_X0-d_J0*d_q0);d_q0=(inv(J0))*d_X0;q0=q0+T*d_q0; 
% d_d_qx=(inv(M_da))*(-D_da*(d_qx-d_q0)-K_da*(qx-q0)+J'*F_ext)+d_d_q0;
% d_qx=d_qx+T*d_d_qx;qx=qx+T*d_qx;
% aa(:,i)=aa(:,i-1)+T*(qx-q);
% tao_star=M*d_d_qx+K*(qx-q)+B*(d_qx-d_q)+L*aa(:,i);
% % data_tao_star(:,i)=tao_star;
% tao=[project([-F,F],tao_star(1)) project([-F,F],tao_star(2))]';
% Xd=[L1*cos(qx(1))+L2*cos(qx(1)+qx(2)) L1*sin(qx(1))+L2*sin(qx(1)+qx(2))]';
% data_q0_to_X0(:,i)=[L1*cos(q0(1))+L2*cos(q0(1)+q0(2)) L1*sin(q0(1))+L2*sin(q0(1)+q0(2))]';
% data_torque_ext(:,i)=J'*F_ext;
% 
% data_sets(1:2,i)=B*(d_qx-d_q); data_sets(4:5,i)=M*d_d_qx;
% data_sets(1:2,i)=K*(qx-q); data_sets(4:5,i)=L*aa(:,i);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 基于微分包含的位控算法 算法2        AdmDIC
% J0=[-L1*sin(q0(1))-L2*sin(q0(1)+q0(2)) -L2*sin(q0(1)+q0(2));L1*cos(q0(1))+L2*cos(q0(1)+q0(2)) L2*cos(q0(1)+q0(2))];
% d_J0=[-L1*cos(q0(1))*d_q0(1)-L2*cos(q0(1)+q0(2))*(d_q0(1)+d_q0(2))  -L2*cos(q0(1)+q0(2))*(d_q0(1)+d_q0(2))
%       -L1*sin(q0(1))*d_q0(1)-L2*sin(q0(1)+q0(2))*(d_q0(1)+d_q0(2))  -L2*sin(q0(1)+q0(2))*(d_q0(1)+d_q0(2))];
% d_d_q0=(inv(J0))*(d_d_X0-d_J0*d_q0);d_q0=(inv(J0))*d_X0;q0=q0+T*d_q0; 
% u_x_star(:,i)=(inv(M_da+D_da*T))*(M_da*u_x(:,i-1)+T*(M_da*d_d_q0+D_da*d_q0+K_da*q0+J'*F_ext));
% % u_x_star(:,i)=(inv(M_da+D_da*T))*(M_da*u_x(:,i-1)+T*(M_da*d_d_q0+D_da*d_q0+K_da*q0+J'*[0 F_ext(2)]'));
% q_x_star=data_qx(:,i)+T*u_x_star(:,i);
% fai_b=B*(data_qx(:,i)-data_q(:,i-1))/T-L*aa(:,i-1);
% fai_a=M*(data_q(:,i)-data_qx(:,i)-T*u_x(:,i-1))/(T*T);
% q_star=data_q(:,i)+(inv(K_hat+M/(T*T)))*(fai_b-fai_a); 
% tao_star=Mat2*((inv(Mat1))*q_x_star-q_star);
% % tao_star=Mat2*((inv(Mat1))*q_x_star-q_star) - J'*F_ext;
% 
% % data_tao_star(:,i)=tao_star;
% tao=[project([-F,F],tao_star(1)) project([-F,F],tao_star(2))]';
% qx=q_star+(inv(K_hat+M/(T*T)))*tao;
% u_x(:,i)=(qx-data_qx(:,i))/T;
% aa(:,i)=aa(:,i-1)+T*(qx-data_q(:,i));
% 
% Xd=[L1*cos(qx(1))+L2*cos(qx(1)+qx(2)) L1*sin(qx(1))+L2*sin(qx(1)+qx(2))]';
% data_q0_to_X0(:,i)=[L1*cos(q0(1))+L2*cos(q0(1)+q0(2)) L1*sin(q0(1))+L2*sin(q0(1)+q0(2))]';
% data_torque_ext(:,i)=J'*F_ext;
% 
%% 基于微分包含的位置滑模控制(双集值)  算法3 用Lambda_1和Lambda_2两个参数 lambda_1=(inv(E+T*Lambda1)); lambda_2=(inv(E+T*Lambda2)); AdmDISMC
AA=lambda_1*h_s*h*(F-F0); BB=lambda_1*h_s*h*(F+F0);
J0=[-L1*sin(q0(1))-L2*sin(q0(1)+q0(2)) -L2*sin(q0(1)+q0(2));L1*cos(q0(1))+L2*cos(q0(1)+q0(2)) L2*cos(q0(1)+q0(2))];
d_J0=[-L1*cos(q0(1))*d_q0(1)-L2*cos(q0(1)+q0(2))*(d_q0(1)+d_q0(2))  -L2*cos(q0(1)+q0(2))*(d_q0(1)+d_q0(2))
      -L1*sin(q0(1))*d_q0(1)-L2*sin(q0(1)+q0(2))*(d_q0(1)+d_q0(2))  -L2*sin(q0(1)+q0(2))*(d_q0(1)+d_q0(2))];
d_d_q0=(inv(J0))*(d_d_X0-d_J0*d_q0);d_q0=(inv(J0))*d_X0;q0=q0+T*d_q0; 
tau_ext=J'*F_ext;    set_1=-tau_ext-(M_x*d_d_q0+D_x*d_q0+K_x*q0);
q1=lambda_1*(qx+h*Lambda1*data_q(:,i)+h_s*M_s*e_r(:,i-1)+h_s*h*f_e_hat);
q2=h_x_star*qx+h_x_star*h_x*(M_x*v_x(:,i-1)-h*set_1);
q3=data_q(:,i)+lambda_2*e_q(:,i-1);
S0(:,1)=[project([-BB(1,1),-AA(1,1)],q3(1)-q1(1)) project([-BB(2,2),-AA(2,2)],q3(2)-q1(2))]';
S0(:,2)=[project([AA(1,1),BB(1,1)],q3(1)-q1(1)) project([AA(2,2),BB(2,2)],q3(2)-q1(2))]';
qx=q1+[project([S0(1,1),S0(1,2)],q2(1)-q1(1)) project([S0(2,1) S0(2,2)],q2(2)-q1(2))]';
q4=data_q(:,i-1)+h_s*M_s*data_d_q(:,i-1)+h_s*h*(tau_ext+f_e_hat);
% q4=data_q(:,i-1)+h_s*M_s*data_d_q(:,i-1)+h_s*h*(-tau_ext*0+f_e_hat);
q5=(inv(h_s*h))*(qx-lambda_2*e_q(:,i-1)-q4);
q6=(inv(lambda_1*h_s*h))*(qx-q1);
% data_q7(:,i)=q7;data_q7_star(:,i)=(h*lambda_h*lambda*q4-q5)/h;
tao_star=q5-q6;
tao=q6+[project([-F0,F0],tao_star(1)) project([-F0,F0],tao_star(2))]';
v_x(:,i)=(qx-data_qx(:,i))/h;
e_q(:,i)=qx-data_q(:,i);
e_r(:,i)=v_x(:,i)+Lambda1*e_q(:,i);

Xd=[L1*cos(qx(1))+L2*cos(qx(1)+qx(2)) L1*sin(qx(1))+L2*sin(qx(1)+qx(2))]';
data_q0_to_X0(:,i)=[L1*cos(q0(1))+L2*cos(q0(1)+q0(2)) L1*sin(q0(1))+L2*sin(q0(1)+q0(2))]';
data_torque_ext(:,i)=J'*F_ext;
% % % % 
data_normal_cone(:,i)=M_da*(((v_x(:,i)-v_x(:,i-1))/T)-d_d_q0)+D_da*((v_x(:,i)-d_q0))+K_da*(qx-q0)-J'*F_ext;
%
%% 力矩作用于二维系统
% C_q=C_q+0.3*[1 0;0 1];
% tau_fric=-0*[sign(d_q(1)) sign(d_q(2))]';
invMq=inv(M_q);
% d_d_q=(invMq)*(-C_q*d_q-g_q+tao+J'*F_ext);
d_d_q=(invMq)*(-C_q*d_q-g_q+g_q+tao+J'*F_ext);      %%%%%%%%%%%%%重力补偿
d_q=d_q+d_d_q*T;
q=q+d_q*T;
end

%% get txt files 
% % datas = [data_X; data_Xd; data_X0; data_tao; data_F_ext];
% datas = [data_normal_cone; data_normal_cone_flag];
% [m, n] = size(datas);
% 
% path_base = 'D:\mechanical_arm_MATLAB_project\MATLAB\MATLAB_Two_DOF_manipulator_admittance_control\result_polt\results_txt\1e+3';
% path_choose = '\data_AdmDIC';
% % method = {'\X.txt', '\X_proxy.txt', '\X_desired.txt', '\tau.txt', '\F_ext.txt'};
% method = {'\data_normal_cone.txt', '\data_normal_cone_flag.txt'};
% 
% for k = 1:2
%     path=char(strcat(path_base,path_choose,method(k)));
%     disp(path);
%     file = fopen(path,'wt');
%     for i = 1:n
%         for j = (2*k-1):(2*k)
%             fprintf(file,'%f',datas(j, i));
%             fprintf(file,'  ');
%         end
%         fprintf(file,'\n');
%     end
%     fclose(file);
% end

%% plot
figure(1)
subplot(2,1,1);
plot(t,data_F_ext(1,:));
leg2=legend('$F_{extX}$');
leg2.Location = 'NorthEast';
hold off;
set(leg2,'Interpreter','latex');
set(leg2,'FontSize',15);
grid on;
xlabel('$t$(s)', 'interpreter','latex', 'FontWeight','bold','FontSize', 15)
ylabel('$F_{ext}$(N)','interpreter','latex', 'FontWeight','bold','FontSize', 15)
set(gca,'FontSize', 15)

subplot(2,1,2);
plot(t,data_F_ext(2,:));
leg2=legend('$F_{extZ}$');
leg2.Location = 'NorthEast';
hold off;
set(leg2,'Interpreter','latex');
set(leg2,'FontSize',15);
grid on;
xlabel('$t$(s)', 'interpreter','latex', 'FontWeight','bold','FontSize', 15)
ylabel('$F_{ext}$(N)','interpreter','latex', 'FontWeight','bold','FontSize', 15)
set(gca,'FontSize', 15)

figure(2)
subplot(2,1,1);
plot(t,data_X(1,:),t,data_Xd(1,:),t,data_X0(1,:));
leg2=legend('$x$','$x_{proxy}$','$x_0$');
leg2.Location = 'NorthEast';
hold off;
set(leg2,'Interpreter','latex');
set(leg2,'FontSize',15);
grid on;
xlabel('$t$(s)', 'interpreter','latex', 'FontWeight','bold','FontSize', 15)
ylabel('$Position$(m)','interpreter','latex', 'FontWeight','bold','FontSize', 15)
set(gca,'FontSize', 15)

subplot(2,1,2);
plot(t,data_X(2,:),t,data_Xd(2,:),t,data_X0(2,:));
leg2=legend('$z$','$z_{proxy}$','$z_0$');
leg2.Location = 'NorthEast';
hold off;
set(leg2,'Interpreter','latex');
set(leg2,'FontSize',15);
grid on;
xlabel('$t$(s)', 'interpreter','latex', 'FontWeight','bold','FontSize', 15)
ylabel('$Position$(m)','interpreter','latex', 'FontWeight','bold','FontSize', 15)
set(gca,'FontSize', 15)

figure(3)
plot(t,data_tao(1,:),t,data_tao(2,:)); 
leg2=legend('$\tau_1$','$\tau_2$');
leg2.Location = 'NorthEast';
hold off;
set(leg2,'Interpreter','latex');
set(leg2,'FontSize',15);
grid on;
xlabel('$t$(s)', 'interpreter','latex', 'FontWeight','bold','FontSize', 15)
ylabel('$\tau$(Nm)','interpreter','latex', 'FontWeight','bold','FontSize', 15)
set(gca,'FontSize', 15)

figure(4)
subplot(2,1,1);
plot(t,data_q(1,:),t,data_qx(1,:),t,data_q0(1,:));
leg2=legend('$q_1$','$q_{x1}$','$q_{01}$');
leg2.Location = 'NorthEast';
hold off;
set(leg2,'Interpreter','latex');
set(leg2,'FontSize',15);
grid on;
xlabel('$t$(s)', 'interpreter','latex', 'FontWeight','bold','FontSize', 15)
ylabel('$Angle_1$(rad)','interpreter','latex', 'FontWeight','bold','FontSize', 15)
set(gca,'FontSize', 15)

subplot(2,1,2);
plot(t,data_q(2,:),t,data_qx(2,:),t,data_q0(2,:));
leg2=legend('$q_2$','$q_{x2}$','$q_{02}$');
leg2.Location = 'NorthEast';
hold off;
set(leg2,'Interpreter','latex');
set(leg2,'FontSize',15);
grid on;
xlabel('$t$(s)', 'interpreter','latex', 'FontWeight','bold','FontSize', 15)
ylabel('$Angle_2$(rad)','interpreter','latex', 'FontWeight','bold','FontSize', 15)
set(gca,'FontSize', 15)

figure(5)
subplot(4,1,1);
plot(t,data_X0(1,:),t,data_q0_to_X0(1,:)); 
leg2=legend('$x_0$','$q_0\,to\,x_0$');
leg2.Location = 'NorthEast';
hold off;
set(leg2,'Interpreter','latex');
set(leg2,'FontSize',15);
grid on;
xlabel('$x$(m)', 'interpreter','latex', 'FontWeight','bold','FontSize', 15)
ylabel('$position$(m)','interpreter','latex', 'FontWeight','bold','FontSize', 15)
set(gca,'FontSize', 15)

subplot(4,1,2);
plot(t,data_X0(2,:),t,data_q0_to_X0(2,:)); 
leg2=legend('$z_0$','$q_0\,to\,z_0$');
leg2.Location = 'NorthEast';
hold off;
set(leg2,'Interpreter','latex');
set(leg2,'FontSize',15);
grid on;
xlabel('$t$(s)', 'interpreter','latex', 'FontWeight','bold','FontSize', 15)
ylabel('$positon$(m)','interpreter','latex', 'FontWeight','bold','FontSize', 15)
set(gca,'FontSize', 15)

subplot(4,1,3);
plot(data_X(1,:),data_X(2,:)); 
leg2=legend('$trajectory$');
leg2.Location = 'NorthEast';
hold off;
set(leg2,'Interpreter','latex');
set(leg2,'FontSize',15);
grid on;
xlabel('$x$(m)', 'interpreter','latex', 'FontWeight','bold','FontSize', 15)
ylabel('$z$(m)','interpreter','latex', 'FontWeight','bold','FontSize', 15)
set(gca,'FontSize', 15)

subplot(4,1,4);
plot(t,data_torque_ext(1,:),t,data_torque_ext(2,:)); 
leg2=legend('$\tau_{ext1}$','$\tau_{ext2}$');
leg2.Location = 'NorthEast';
hold off;
set(leg2,'Interpreter','latex');
set(leg2,'FontSize',15);
grid on;
xlabel('$t$(s)', 'interpreter','latex', 'FontWeight','bold','FontSize', 15)
ylabel('$torque_{ext}$(Nm)','interpreter','latex', 'FontWeight','bold','FontSize', 15)
set(gca,'FontSize', 15)

% figure(6)
% subplot(2,1,1);
% plot(t,data_tao(1,:),t,data_tao(2,:)); 
% leg2=legend('$\tau_1$','$\tau_2$');
% leg2.Location = 'NorthEast';
% hold off;
% set(leg2,'Interpreter','latex');
% set(leg2,'FontSize',15);
% grid on;
% xlabel('$t$(s)', 'interpreter','latex', 'FontWeight','bold','FontSize', 15)
% ylabel('$\tau$(Nm)','interpreter','latex', 'FontWeight','bold','FontSize', 15)
% set(gca,'FontSize', 15)
% 
% subplot(2,1,2);
% plot(t,data_F_ext(1,:),t,data_F_ext(2,:));
% leg2=legend('$F_{extX}$','$F_{extZ}$');
% leg2.Location = 'NorthEast';
% hold off;
% set(leg2,'Interpreter','latex');
% set(leg2,'FontSize',15);
% grid on;
% xlabel('$t$(s)', 'interpreter','latex', 'FontWeight','bold','FontSize', 15)
% ylabel('$F_{ext}$(N)','interpreter','latex', 'FontWeight','bold','FontSize', 15)
% set(gca,'FontSize', 15)

figure(6)
subplot(3,2,[1,2]);
plot(t,data_X(1,:),t,data_Xd(1,:),t,data_X0(1,:));
leg2=legend('$x$','$x_{prx}$','$x_0$');
leg2.Location = 'NorthEast';
hold off;
set(leg2,'Interpreter','latex');
set(leg2,'FontSize',12);
% grid on;
xlabel('$t$(s)', 'interpreter','latex', 'FontWeight','bold','FontSize', 10)
ylabel('$Position$(m)','interpreter','latex', 'FontWeight','bold','FontSize', 10)
set(gca,'FontSize', 10)

subplot(3,2,[3,4]);
plot(t,data_X(2,:),t,data_Xd(2,:),t,data_X0(2,:));
leg2=legend('$y$','$y_{prx}$','$y_0$');
leg2.Location = 'NorthEast';
hold off;
set(leg2,'Interpreter','latex');
set(leg2,'FontSize',12);
% grid on;
xlabel('$t$(s)', 'interpreter','latex', 'FontWeight','bold','FontSize', 10)
ylabel('$Position$(m)','interpreter','latex', 'FontWeight','bold','FontSize', 10)
set(gca,'FontSize', 10)

subplot(3,2,5);
plot(t,data_F_ext(1,:),t,data_F_ext(2,:));
leg2=legend('$f_{ext,\,x}$','$f_{ext,\,y}$');
leg2.Location = 'NorthEast';
hold off;
set(leg2,'Interpreter','latex');
set(leg2,'FontSize',12);
% grid on;
xlabel('$t$(s)', 'interpreter','latex', 'FontWeight','bold','FontSize', 10)
ylabel('$f_{ext}$(N)','interpreter','latex', 'FontWeight','bold','FontSize', 10)
set(gca,'FontSize', 10)

subplot(3,2,6);
plot(t,data_tao(1,:),t,data_tao(2,:)); 
leg2=legend('$\tau_1$','$\tau_2$');
leg2.Location = 'NorthEast';
hold off;
set(leg2,'Interpreter','latex');
set(leg2,'FontSize',12);
% grid on;
xlabel('$t$(s)', 'interpreter','latex', 'FontWeight','bold','FontSize', 10)
ylabel('$\tau$(Nm)','interpreter','latex', 'FontWeight','bold','FontSize', 10)
set(gca,'FontSize', 10)


% figure(7)
% subplot(2,1,1);
% plot(t,data_normal_cone(1,:),t,data_normal_cone(2,:)); 
% leg2=legend('${\cal N}_{{\cal F},1}(\tau)$','${\cal N}_{{\cal F},2}(\tau)$');
% leg2.Location = 'NorthEast';
% hold off;
% set(leg2,'Interpreter','latex');
% set(leg2,'FontSize',15);
% grid on;
% xlabel('$t$(s)', 'interpreter','latex', 'FontWeight','bold','FontSize', 15)
% ylabel('${\cal N}_{{\cal F}}(\tau)$(Nm)','interpreter','latex', 'FontWeight','bold','FontSize', 15)
% set(gca,'FontSize', 15)
% 
% subplot(2,1,2);
% plot(t,data_normal_cone_flag(1,:),t,data_normal_cone_flag(2,:)); 
% leg2=legend('${\cal N}_{{\cal F},1}(\tau)$','${\cal N}_{{\cal F},2}(\tau)$');
% leg2.Location = 'NorthEast';
% hold off;
% set(leg2,'Interpreter','latex');
% set(leg2,'FontSize',15);
% grid on;
% xlabel('$t$(s)', 'interpreter','latex', 'FontWeight','bold','FontSize', 15)
% ylabel('$\rm{Flag} \ \ {\cal N}_{{\cal F}}(\tau)$','interpreter','latex', 'FontWeight','bold','FontSize', 15)
% set(gca,'FontSize', 15)


toc;
