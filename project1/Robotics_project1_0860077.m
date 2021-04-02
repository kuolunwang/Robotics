%0860077 王國倫
number = input('1.Forward kinematic\n2.Inverse kinematic\n3.exit\nPlease input (1~3):');

degree_to_rad = pi/180;  %transfer degree to rad
rad_to_degree = 180/pi;  %transfer rad to degree
valid = true;  %make sure every input joint in the range

%Forward kinematic
if number == 1
    
    %kinematic table parameter
    a2 = 0.432;    
    a3 = -0.02;    
    d3 = 0.149;    
    d4 = 0.433;     
    
    %input six joint variables
    theta = input('\ninput six joint variables:[theta1,theta2,theta3,theta4,theta5,theta6]\n');
    theta1=theta(1)*degree_to_rad;
    theta2=theta(2)*degree_to_rad;
    theta3=theta(3)*degree_to_rad;
    theta4=theta(4)*degree_to_rad;
    theta5=theta(5)*degree_to_rad;
    theta6=theta(6)*degree_to_rad;
    
    % verify every joint degree
    while abs(theta(1))>160
        fprintf('Theat1 out of range\n');
        valid =false;
        break
    end
    
    while abs(theta(2))>125
        fprintf('Theat1 out of range\n');
        valid =false;
        break
    end
    
    while abs(theta(3))>135
        fprintf('Theat1 out of range\n');
        valid =false;
        break
    end
    
    while abs(theta(4))>140
        fprintf('Theat1 out of range\n');
        valid =false;
        break
    end
    
    while abs(theta(5))>100
        fprintf('Theat1 out of range\n');
        valid =false;
        break
    end
    
    while abs(theta(6))>260
        fprintf('Theat6 out of range\n');
        valid =false;
        break
    end
    
    if valid
     
        %transformation matrix A1~A6    
        A1 = [ cos(theta1)   0    -sin(theta1)    0;
               sin(theta1)   0     cos(theta1)    0;
                         0  -1              0     0;
                         0   0              0     1  ];
                     
        A2 = [ cos(theta2)   -sin(theta2)    0    a2*cos(theta2);
               sin(theta2)    cos(theta2)    0    a2*sin(theta2);
                         0              0    1                 0;
                         0              0    0                 1  ];
        
        A3 = [ cos(theta3)   0     sin(theta3)   a3*cos(theta3);
               sin(theta3)   0    -cos(theta3)   a3*sin(theta3);
                         0   1              0                d3;
                         0   0              0                 1  ];
                     
        A4 = [ cos(theta4)     0    -sin(theta4)     0;
               sin(theta4)     0     cos(theta4)     0;
                         0    -1              0     d4;
                         0     0              0      1  ];
                     
        A5 = [ cos(theta5)     0     sin(theta5)      0;
               sin(theta5)     0    -cos(theta5)      0;
                         0     1              0       0;
                         0     0              0       1  ];
                     
        A6 = [ cos(theta6)   -sin(theta6)     0     0;
               sin(theta6)    cos(theta6)     0     0;
                         0              0     1     0;
                         0              0     0     1  ];
        
        % calculate final transformation matrix
        T6=A1*A2*A3*A4*A5*A6;
        
        % get n o a p value
        nx = T6(1,1);
        ny = T6(2,1);
        nz = T6(3,1);
        
        ox = T6(1,2);
        oy = T6(2,2);
        oz = T6(3,2);
        
        ax = T6(1,3);
        ay = T6(2,3);
        az = T6(3,3);
        
        px = T6(1,4);
        py = T6(2,4);
        pz = T6(3,4);
            
        % transfer n o a to phi theta psi
        phi=atan2(ay,ax)*rad_to_degree;               
        theta=atan2(sqrt((ax)^2+(ay)^2),az)*rad_to_degree;
        psi=atan2(oz,-nz)*rad_to_degree;
        
        % output result
        disp('(n a o p)');
        disp(T6);
        
        disp('Cartesian point')
        fprintf('(x,y,z,phi,theta,phi) = (%f ,%f ,%f , %f ,%f ,%f )\n',px,py,pz,phi,theta,psi)
        
    else 
        disp('error!')
            
    end
%Inverse Kinematic
elseif number == 2
    % input n o a p transformation matrix
    T=input('\nT=[nx ox ax px ;  ny oy ay py ; nz oz az pz ;0 0 0 1;]\n');  
    
    %kinematic table parameter
    a2 = 0.432;    
    a3 = -0.02;    
    d3 = 0.149;    
    d4 = 0.433; 
    
    %get n o a p value
    nx = T(1,1);
    ny = T(2,1);
    nz = T(3,1);
        
    ox = T(1,2);
    oy = T(2,2);
    oz = T(3,2);
        
    ax = T(1,3);
    ay = T(2,3);
    az = T(3,3);
        
    px = T(1,4);
    py = T(2,4);
    pz = T(3,4);
    
    %calculate theta1
    theta1= [atan2(py,px)-atan2(d3,sqrt(px^2+py^2-d3^2)) atan2(py,px)-atan2(d3,-1*sqrt(px^2+py^2-d3^2));];
    
    M =((px)^2+(py)^2+(pz)^2-(a2)^2-(a3)^2-(d3)^2-(d4)^2)/(2*(a2));
    
    %calculate theta3
    theta3 = [-atan2(a3,d4)+atan2(M,sqrt(d4^2+a3^2-M^2)) -atan2(a3,d4)+atan2(M,-1*sqrt(d4^2+a3^2-M^2));];
    
    %calculate theta2 4 5 6
    for i = 1:2
        for j = 1:2
            
            %calculate theta2
            c1 = cos(theta1(j));
            s1 = sin(theta1(j));
            c3 = cos(theta3(i));
            s3 = sin(theta3(i));
            v1 = c1*px+s1*py;
            v2 = a3+a2*c3;
            v3 = d4+a2*s3;
            s23_t = v1*v3-v2*pz;
            c23_t = v1*v2+v3*pz;
            t23 = atan2(s23_t,c23_t);
            theta2 = t23 - theta3(i);
            
            %calculate theta4,5,6
            c23 = cos(t23);
            s23 = sin(t23);
            c4s5 = c1*c23*ax + s1*c23*ay - s23*az;
            s4s5 = -s1*ax + c1*ay;
            c5 = c1*s23*ax + s1*s23*ay + c23*az;
            s5c6 = -1*(c1*s23*nx+s1*s23*ny+c23*nz);
            s5s6 = (c1*s23*ox+s1*s23*oy+c23*oz);
            
            % case 1 theta 4 5 6
            theta4 = atan2(s4s5, c4s5);
            theta5 = acos(c5);
            theta6 = atan2(s5s6, s5c6);
            
            r2a(theta1(j),theta2,theta3(i),theta4,theta5,theta6);
            
            % case 2 theta 4 5 6
            theta4 = atan2(-s4s5, -c4s5);
            theta5 = -acos(c5);
            theta6 = atan2(-s5s6, -s5c6);

            r2a(theta1(j),theta2,theta3(i),theta4,theta5,theta6);
        end
    end

elseif number == 3
    fprintf('exit!\n');    
end

% transfer rad to degree, verify its angle limitation, and output result
function r2a(theta1,theta2,theta3,theta4,theta5,theta6)
    
        rad_to_degree = 180/pi;  %transfer rad to degree
        
        % transfer every joint to degree
        angle=[];
        angle(1)=theta1*rad_to_degree;
        angle(2)=theta2*rad_to_degree;
        angle(3)=theta3*rad_to_degree;
        angle(4)=theta4*rad_to_degree;
        angle(5)=theta5*rad_to_degree;
        angle(6)=theta6*rad_to_degree;
        
        disp('(θ1,θ2,θ3,θ4,θ5,θ6) =')
        
        % make sure joint degree in the range
        if abs(angle(1))>160
            fprintf("theta1 is out of range!\n");
        end
        if abs(angle(2))>125
            fprintf("theta2 is out of range!\n");
        end
        if abs(angle(3))>135
            fprintf("theta3 is out of range!\n");
        end
        if abs(angle(4))>140
            fprintf("theta4 is out of range!\n");
        end
        if abs(angle(5))>100
            fprintf("theta5 is out of range!\n");
        end
        if abs(angle(6))>260
            fprintf("theta6 is out of range!\n");
        end
        
        % output result
        disp([angle(1),angle(2),angle(3),angle(4),angle(5),angle(6)]);
end
    

