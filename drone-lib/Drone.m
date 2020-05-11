classdef Drone < handle
%%MEMBERS
    properties
        g
        t
        dt
        tf
        
        m
        l
        I
        
        x           %[X,Y,Z,phi,theta,psi,Xdot,Ydot,zdot,p,q,r]'
        r           %[X,Y,Z]'
        rdot        %[Xdot,Ydot,Zdot]'
        euler       %[phi,theta,psi]'
        w           %[p,q,r]'
        
        xdot
        
        u           %[T,M1,M2,M3]' (inputs U1,U2,U3,U4)
        T           %T_sum
        M           %[M1,M2,M3]'
    end
    
    properties
        phi_des
        phi_err
        phi_err_prev
        phi_err_sum
        
        theta_des
        theta_err
        theta_err_prev
        theta_err_sum
        
        psi_des
        psi_err
        psi_err_prev
        psi_err_sum
        
        zdot_des
        zdot_err
        zdot_err_prev
        zdot_err_sum
        
        kP_phi
        kI_phi
        kD_phi
        
        kP_theta
        kI_theta
        kD_theta
        
        kP_psi
        kI_psi
        kD_psi
        
        kP_zdot
        kI_zdot
        kD_zdot
    end
    
%%METHODS
    methods
        %%CONSTRUCTOR
        function obj = Drone(params, initStates, initInputs, gains, simTime)
            obj.g=9.81;
            obj.t=0.0;
            obj.dt=0.01;
            obj.tf=simTime;
            
            obj.m=params('mass');
            obj.l=params('armLength');
            obj.I=[params('Ixx'),-params('Ixy'),-params('Ixz');...
                -params('Ixy'),params('Iyy'),-params('Iyz');...
                -params('Ixz'),-params('Iyz'),params('Izz')];
            
            obj.x=initStates;
            obj.r=obj.x(1:3);
            obj.rdot=obj.x(7:9);
            obj.euler=obj.x(4:6);
            obj.w=obj.x(10:12);
            
            obj.xdot=zeros(12,1);
            
            obj.u=initInputs;
            obj.T=obj.u(1);
            obj.M=obj.u(2:4);
            
            obj.phi_des=0.0;
            obj.phi_err=0.0;
            obj.phi_err_prev=0.0;
            obj.phi_err_sum=0.0;

            obj.theta_des=0.0;
            obj.theta_err=0.0;
            obj.theta_err_prev=0.0;
            obj.theta_err_sum=0.0;

            obj.psi_des=0.0;
            obj.psi_err=0.0;
            obj.psi_err_prev=0.0;
            obj.psi_err_sum=0.0;

            obj.zdot_des=0.0;
            obj.zdot_err=0.0;
            obj.zdot_err_prev=0.0;
            obj.zdot_err_sum=0.0;

            obj.kP_phi=gains('P_phi');
            obj.kI_phi=gains('I_phi');
            obj.kD_phi=gains('D_phi');

            obj.kP_theta=gains('P_theta');
            obj.kI_theta=gains('I_theta');
            obj.kD_theta=gains('D_theta');

            obj.kP_psi=gains('P_psi');
            obj.kI_psi=gains('I_psi');
            obj.kD_psi=gains('D_psi');

            obj.kP_zdot=gains('P_zdot');
            obj.kI_zdot=gains('I_zdot');
            obj.kD_zdot=gains('D_zdot');
        end
           
        
        function state=GetState(obj)
            state=obj.x;
        end
        
        
        function obj=EvalEOM(obj)
            bRi=RPY2Rot(obj.euler);
            R=bRi';
            
            obj.xdot(1:3)=obj.rdot; %x(7:9)
            obj.xdot(7:9)=1/obj.m*([0; 0; obj.m*obj.g]+R*obj.T*[0;0;-1]);
            
            phi=obj.euler(1); theta = obj.euler(2);
            obj.xdot(4:6)=[ 1 sin(phi)*tan(theta) cos(phi)*tan(theta);
                            0 cos(phi)              -sin(phi);
                            0 sin(phi)*sec(theta) cos(phi)*sec(theta)]*obj.w;
            obj.xdot(10:12)=(obj.I) \ (obj.M - cross(obj.w, obj.I * obj.w));
        end
        
        function obj=UpdateState(obj)
            obj.t=obj.t+obj.dt;
            
            obj.EvalEOM();
            obj.x=obj.x+obj.xdot.*obj.dt;
            
            obj.r=obj.x(1:3);
            obj.rdot=obj.x(7:9);
            obj.euler=obj.x(4:6);
            obj.w=obj.x(10:12);
        end
        
        %% CONTROLLER
        function obj=AttitudeCtrl(obj, refSig)
            obj.u(1)=obj.m*obj.g;
            obj.u(2)=0.0;
            obj.u(3)=0.0;
            obj.u(4)=0.1;
            
            
            obj.phi_des=refSig(1);
            obj.theta_des=refSig(2);
            obj.psi_des=refSig(3);
            obj.zdot_des=refSig(4);
            
            obj.phi_err=obj.phi_des-obj.euler(1);
            obj.theta_err=obj.theta_des-obj.euler(2);
            obj.psi_err=obj.psi_des-obj.euler(3);
            obj.zdot_err= obj.zdot_des-obj.rdot(3);
            
            obj.u(2) = (obj.kP_phi*obj.phi_err +...
                        obj.kI_phi*obj.phi_err_sum +....
                        obj.kD_phi*(obj.phi_err-obj.phi_err_prev)/obj.dt);
                        %obj.kD_phi*(0-obj.w(1)); %Use this in real life
                        %because differentiation as used above is not
                        %recommended
                    
            obj.phi_err_sum=obj.phi_err_sum+obj.phi_err;
            obj.phi_err_prev=obj.phi_err;
            
            obj.u(3) = (obj.kP_theta*obj.theta_err +...
                        obj.kI_theta*obj.theta_err_sum +....
                        obj.kD_theta*(obj.theta_err-obj.theta_err_prev)/obj.dt);
                    
            obj.theta_err_sum=obj.theta_err_sum+obj.theta_err;
            obj.theta_err_prev=obj.theta_err;
            
            obj.u(4) = (obj.kP_psi*obj.psi_err +...
                        obj.kI_psi*obj.psi_err_sum +....
                        obj.kD_psi*(obj.psi_err-obj.psi_err_prev)/obj.dt);
                    
            obj.psi_err_sum=obj.psi_err_sum+obj.psi_err;
            obj.psi_err_prev=obj.psi_err;
            
            obj.u(1) = obj.m*obj.g - (obj.kP_zdot*obj.zdot_err +...
                        obj.kI_zdot*obj.zdot_err_sum +....
                        obj.kD_zdot*(obj.zdot_err-obj.zdot_err_prev)/obj.dt);
                    
            obj.zdot_err_sum=obj.zdot_err_sum+obj.zdot_err;
            obj.zdot_err_prev=obj.zdot_err;
            
            %obj.u(2:4)=[0;0;0];
            
            obj.T=obj.u(1);
            obj.M=obj.u(2:4);
        end
        
    end

end