
disp('===================================================================');
disp('This framework generates JavaScript code to run on Nodejs platform');
disp('The framework uses the State-space model of the system to generate the MPC');
disp('===================================================================');

ss_sys = input('Please, Enter the State-space model of the system: ');
Ts = input('Enter the value of the sampling time (Ts): ');
u_min = input('Set the lower bounds on the Manipulated Variables (MV): ');
u_max = input('Set the upper bounds on the Manipulated Variables (MV): ');
delta_U_min = input('Set the lower bounds on the interval-to-interval change for the MV: ');
delta_U_max = input('Set the upper bounds on the interval-to-interval change for the MV: ');
y_min = input('Set the lower bounds on the Output Variables (OV): ');
y_max = input('Set the upper bounds on the Output Variables (OV): ');

Np = input('Enter the value of Prediction Horizon (Np): ');
Nc = input('Enter the value of Control Horizon (Nc): ');
ref = input('Enter the refernce (Set-point): ');

Y_N = input('Do you run your system on Intel Galileo Development Board?(y/n) ','s');
if Y_N == 'y'
    dpYN = input ('will you use digital pins? (y/n) ','s');
    if dpYN == 'y'
    disp('Choose the digital pins that you used');
    dp_num = input('How many digital pins will you use? (1-14)');
    if dp_num > 0 && dp_num <= 13
        pin = 1;
       while pin <= dp_num
           fprintf('Enter the pin number %d : (0-13) ', pin);
           d_pin(pin) = input('');
           pin = pin +1;
       end
    else
        disp('!!Undefined pin on the board!!');
    end
    elseif dpYN == 'n'
        disp('You did not select any digital pins!! ');
        d_pin = 6;
    else
        disp('!!Undefined input!!');
        d_pin = 6;
    end
    apYN = input ('will you use analog pins? (y/n) ','s');
    if apYN == 'y'
    disp('Choose the analog pins that you used');
    ap_num = input('How many analog pins will you use? (1-6)');
    if ap_num > 0 && ap_num <= 5
        pin = 1;
       while pin <= ap_num
           fprintf('Enter the pin number %d : (0-5) ', pin);
           a_pin(pin) = input('');
           pin = pin +1;
       end
    else
        disp('!!Undefined pin on the board!!');
    end
    elseif apYN == 'n'
        disp('You did not select any analog pins!! ');
        a_pin = 0;
    else
        disp('!!Undefined input!!');
        a_pin = 0;
    end
    kFilter = input('Do you want to use Kalamn Filter?(y/n) ','s');
    if kFilter == 'y'
        varProcess = input('Enter the value of the process variance (Q): ');
        varVolt = input('Enter the value of the variance of sensor measuerments (R): ');
    elseif kFilter == 'n'
        disp('You did not use Kalman Filter!! ');
        varProcess = 1e-7;
        varVolt    = 1.12e-5;
    else
        disp('!!Undefined input!!');
        varProcess = 1e-7;
        varVolt    = 1.12e-5;
        kFilter = 'n';
    end
    
elseif Y_N == 'n'
    disp('You did not select any digital or analog pins!! ');
    Y_N = 'n';
    KFilter = 'n';
    d_pin = 6;
    a_pin = 0;
    varProcess = 1e-7;
    varVolt    = 1.12e-5;
else
    disp('!!Undefined input!!');
    Y_N = 'n';
    KFilter = 'n';
    d_pin = 6;
    a_pin = 0;
    varProcess = 1e-7;
    varVolt    = 1.12e-5;
end

[Gs,TT]= ssbal(ss_sys);
Gsd = c2d(Gs,Ts);
A = Gsd.A;
B = Gsd.B;
C = Gsd.C;
n = length(A);
k = size(B,2);
m = size(C,1);

if Y_N == 'y' && KFilter == 'y'
    % Generating JavaScript file
    fileID = fopen('main.js','w');
    % Write comments at the top of js file
    fprintf(fileID,'%s \n','// Real-time MPC controller on Galileo with Kalman Filter');
    fprintf(fileID,'%s \n','//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=//');
    fprintf(fileID,'%s \n','// Initializing of MPC ');

    % Matrix Ap
    fprintf(fileID,'%s','var Ap = [[');
    for r = 1:n
        for c = 1:n
            fprintf(fileID,'%4.4f',A(r,c));
            if c < n
                fprintf(fileID,'%s',' , ');
            end
        end
        if r < n
        fprintf(fileID,'%s','], [');
        end
    end
    fprintf(fileID,'%s\n',']];');
    % Matrix Bp
    fprintf(fileID,'%s','var Bp = [[');
    for r = 1:n
        for c = 1:k
            fprintf(fileID,'%4.4f',B(r,c));
            if c < k
                fprintf(fileID,'%s',' , ');
            end
        end
        if r < n
        fprintf(fileID,'%s','], [');
        end
    end
    fprintf(fileID,'%s\n',']];');
    % Matrix C
    fprintf(fileID,'%s','var Cp = [[');
    for r = 1:m
        for c = 1:n
            fprintf(fileID,'%4.4f',C(r,c));
            if c < n
                fprintf(fileID,'%s',' , ');
            end
        end
        if r < m
        fprintf(fileID,'%s','], [');
        end
    end
    fprintf(fileID,'%s\n',']];');
    % Rest constants
    fprintf(fileID,'%s %d %s\n','var n =',n, ';');
    fprintf(fileID,'%s %d %s\n','var k =',k, ';');
    fprintf(fileID,'%s %d %s\n','var m =',m, ';');
    fprintf(fileID,'%s %d %s\n','var Np =',Np, ';');
    fprintf(fileID,'%s %d %s\n','var Nc =',Nc, ';');
    fprintf(fileID,'%s %d %s\n','var u_min =',u_min, ';');
    fprintf(fileID,'%s %d %s\n','var u_max =',u_max, ';');
    fprintf(fileID,'%s %d %s\n','var delta_U_min =',delta_U_min, ';');
    fprintf(fileID,'%s %d %s\n','var delta_U_max =',delta_U_max, ';');
    fprintf(fileID,'%s %d %s\n','var y_min =',y_min, ';');
    fprintf(fileID,'%s %d %s\n','var y_max =',y_max, ';');
    fprintf(fileID,'%s %d %s\n','var ref =',ref, ';');

    for i = 1:length(d_pin)
        fprintf(fileID,'%s%d %s %d %s\n','var pwmPin',i,'=',d_pin(i), ';');
    end
    for i = 1:length(a_pin)
        fprintf(fileID,'%s%d %s %d %s\n','var SensorPin',i,'=',a_pin(i), ';');
    end

    fprintf(fileID,'%s %d %s\n','var varProcess =',varProcess, ';');
    fprintf(fileID,'%s %d %s\n','var varVolt =',varVolt, ';');
    fclose(fileID);

    % append the mpc_init file with the main file
    fr = fopen( 'mpc_galileo_filter.js', 'rt' );
    fw = fopen( 'main.js', 'at' );
    while feof( fr ) == 0
        tline = fgetl( fr );
        fwrite( fw, sprintf('%s\n',tline ) );
    end
    fclose(fr);
    fclose(fw);
    
elseif Y_N == 'n' && KFilter == 'n'
    % Generating JavaScript file
    fileID = fopen('main.js','w');
    % Write comments at the top of js file
    fprintf(fileID,'%s \n','// Real-time MPC controller');
    fprintf(fileID,'%s \n','//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=//');
    fprintf(fileID,'%s \n','// Initializing of MPC ');

    % Matrix Ap
    fprintf(fileID,'%s','var Ap = [[');
    for r = 1:n
        for c = 1:n
            fprintf(fileID,'%4.4f',A(r,c));
            if c < n
                fprintf(fileID,'%s',' , ');
            end
        end
        if r < n
        fprintf(fileID,'%s','], [');
        end
    end
    fprintf(fileID,'%s\n',']];');
    % Matrix Bp
    fprintf(fileID,'%s','var Bp = [[');
    for r = 1:n
        for c = 1:k
            fprintf(fileID,'%4.4f',B(r,c));
            if c < k
                fprintf(fileID,'%s',' , ');
            end
        end
        if r < n
        fprintf(fileID,'%s','], [');
        end
    end
    fprintf(fileID,'%s\n',']];');
    % Matrix C
    fprintf(fileID,'%s','var Cp = [[');
    for r = 1:m
        for c = 1:n
            fprintf(fileID,'%4.4f',C(r,c));
            if c < n
                fprintf(fileID,'%s',' , ');
            end
        end
        if r < m
        fprintf(fileID,'%s','], [');
        end
    end
    fprintf(fileID,'%s\n',']];');
    % Rest constants
    fprintf(fileID,'%s %d %s\n','var n =',n, ';');
    fprintf(fileID,'%s %d %s\n','var k =',k, ';');
    fprintf(fileID,'%s %d %s\n','var m =',m, ';');
    fprintf(fileID,'%s %d %s\n','var Np =',Np, ';');
    fprintf(fileID,'%s %d %s\n','var Nc =',Nc, ';');
    fprintf(fileID,'%s %d %s\n','var u_min =',u_min, ';');
    fprintf(fileID,'%s %d %s\n','var u_max =',u_max, ';');
    fprintf(fileID,'%s %d %s\n','var delta_U_min =',delta_U_min, ';');
    fprintf(fileID,'%s %d %s\n','var delta_U_max =',delta_U_max, ';');
    fprintf(fileID,'%s %d %s\n','var y_min =',y_min, ';');
    fprintf(fileID,'%s %d %s\n','var y_max =',y_max, ';');
    fprintf(fileID,'%s %d %s\n','var ref =',ref, ';');
    
    fclose(fileID);

    % append the mpc_init file with the main file
    fr = fopen( 'mpc.js', 'rt' );
    fw = fopen( 'main.js', 'at' );
    while feof( fr ) == 0
        tline = fgetl( fr );
        fwrite( fw, sprintf('%s\n',tline ) );
    end
    fclose(fr);
    fclose(fw);
    
elseif Y_N == 'y'
    % Generating JavaScript file
    fileID = fopen('main.js','w');
    % Write comments at the top of js file
    fprintf(fileID,'%s \n','// Real-time MPC controller on Galileo');
    fprintf(fileID,'%s \n','//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=//');
    fprintf(fileID,'%s \n','// Initializing of MPC ');

    % Matrix Ap
    fprintf(fileID,'%s','var Ap = [[');
    for r = 1:n
        for c = 1:n
            fprintf(fileID,'%4.4f',A(r,c));
            if c < n
                fprintf(fileID,'%s',' , ');
            end
        end
        if r < n
        fprintf(fileID,'%s','], [');
        end
    end
    fprintf(fileID,'%s\n',']];');
    % Matrix Bp
    fprintf(fileID,'%s','var Bp = [[');
    for r = 1:n
        for c = 1:k
            fprintf(fileID,'%4.4f',B(r,c));
            if c < k
                fprintf(fileID,'%s',' , ');
            end
        end
        if r < n
        fprintf(fileID,'%s','], [');
        end
    end
    fprintf(fileID,'%s\n',']];');
    % Matrix C
    fprintf(fileID,'%s','var Cp = [[');
    for r = 1:m
        for c = 1:n
            fprintf(fileID,'%4.4f',C(r,c));
            if c < n
                fprintf(fileID,'%s',' , ');
            end
        end
        if r < m
        fprintf(fileID,'%s','], [');
        end
    end
    fprintf(fileID,'%s\n',']];');
    % Rest constants
    fprintf(fileID,'%s %d %s\n','var n =',n, ';');
    fprintf(fileID,'%s %d %s\n','var k =',k, ';');
    fprintf(fileID,'%s %d %s\n','var m =',m, ';');
    fprintf(fileID,'%s %d %s\n','var Np =',Np, ';');
    fprintf(fileID,'%s %d %s\n','var Nc =',Nc, ';');
    fprintf(fileID,'%s %d %s\n','var u_min =',u_min, ';');
    fprintf(fileID,'%s %d %s\n','var u_max =',u_max, ';');
    fprintf(fileID,'%s %d %s\n','var delta_U_min =',delta_U_min, ';');
    fprintf(fileID,'%s %d %s\n','var delta_U_max =',delta_U_max, ';');
    fprintf(fileID,'%s %d %s\n','var y_min =',y_min, ';');
    fprintf(fileID,'%s %d %s\n','var y_max =',y_max, ';');
    fprintf(fileID,'%s %d %s\n','var ref =',ref, ';');

    for i = 1:length(d_pin)
        fprintf(fileID,'%s%d %s %d %s\n','var pwmPin',i,'=',d_pin(i), ';');
    end
    for i = 1:length(a_pin)
        fprintf(fileID,'%s%d %s %d %s\n','var SensorPin',i,'=',a_pin(i), ';');
    end

    fclose(fileID);

    % append the mpc_init file with the main file
    fr = fopen( 'mpc_galileo_filter.js', 'rt' );
    fw = fopen( 'main.js', 'at' );
    while feof( fr ) == 0
        tline = fgetl( fr );
        fwrite( fw, sprintf('%s\n',tline ) );
    end
    fclose(fr);
    fclose(fw);
    
else
    % Generating JavaScript file
    fileID = fopen('main.js','w');
    % Write comments at the top of js file
    fprintf(fileID,'%s \n','// Real-time MPC controller on Galileo with Kalman Filter');
    fprintf(fileID,'%s \n','//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=//');
    fprintf(fileID,'%s \n','// Initializing of MPC ');

    % Matrix Ap
    fprintf(fileID,'%s','var Ap = [[');
    for r = 1:n
        for c = 1:n
            fprintf(fileID,'%4.4f',A(r,c));
            if c < n
                fprintf(fileID,'%s',' , ');
            end
        end
        if r < n
        fprintf(fileID,'%s','], [');
        end
    end
    fprintf(fileID,'%s\n',']];');
    % Matrix Bp
    fprintf(fileID,'%s','var Bp = [[');
    for r = 1:n
        for c = 1:k
            fprintf(fileID,'%4.4f',B(r,c));
            if c < k
                fprintf(fileID,'%s',' , ');
            end
        end
        if r < n
        fprintf(fileID,'%s','], [');
        end
    end
    fprintf(fileID,'%s\n',']];');
    % Matrix C
    fprintf(fileID,'%s','var Cp = [[');
    for r = 1:m
        for c = 1:n
            fprintf(fileID,'%4.4f',C(r,c));
            if c < n
                fprintf(fileID,'%s',' , ');
            end
        end
        if r < m
        fprintf(fileID,'%s','], [');
        end
    end
    fprintf(fileID,'%s\n',']];');
    % Rest constants
    fprintf(fileID,'%s %d %s\n','var n =',n, ';');
    fprintf(fileID,'%s %d %s\n','var k =',k, ';');
    fprintf(fileID,'%s %d %s\n','var m =',m, ';');
    fprintf(fileID,'%s %d %s\n','var Np =',Np, ';');
    fprintf(fileID,'%s %d %s\n','var Nc =',Nc, ';');
    fprintf(fileID,'%s %d %s\n','var u_min =',u_min, ';');
    fprintf(fileID,'%s %d %s\n','var u_max =',u_max, ';');
    fprintf(fileID,'%s %d %s\n','var delta_U_min =',delta_U_min, ';');
    fprintf(fileID,'%s %d %s\n','var delta_U_max =',delta_U_max, ';');
    fprintf(fileID,'%s %d %s\n','var y_min =',y_min, ';');
    fprintf(fileID,'%s %d %s\n','var y_max =',y_max, ';');
    fprintf(fileID,'%s %d %s\n','var ref =',ref, ';');

    for i = 1:length(d_pin)
        fprintf(fileID,'%s%d %s %d %s\n','var pwmPin',i,'=',d_pin(i), ';');
    end
    for i = 1:length(a_pin)
        fprintf(fileID,'%s%d %s %d %s\n','var SensorPin',i,'=',a_pin(i), ';');
    end

    fprintf(fileID,'%s %d %s\n','var varProcess =',varProcess, ';');
    fprintf(fileID,'%s %d %s\n','var varVolt =',varVolt, ';');
    fclose(fileID);

    % append the mpc_init file with the main file
    fr = fopen( 'mpc_galileo_filter.js', 'rt' );
    fw = fopen( 'main.js', 'at' );
    while feof( fr ) == 0
        tline = fgetl( fr );
        fwrite( fw, sprintf('%s\n',tline ) );
    end
    fclose(fr);
    fclose(fw);
end