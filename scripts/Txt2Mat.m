%% Importar archivos de .txt a .mat
close all
clc
clear variables
tic
%%
% Se registran los ensayos que ya se han importado
Origen = pwd;
cd('ECARM MATLAB')
archivos = dir(fullfile(pwd,'*_Ensayo*'));
archivos_reg = strings(length(archivos),1);
for k=1:length(archivos)
    archivos_reg(k,1) = cellstr(archivos(k).name);
end
cd ..

% Se determinan las carpetas en el nivel de clasificación por años
nivel1 = dir(fullfile(pwd,'ECARM 2*'));
N0 = length(nivel1);
carpeta_raiz = strings(N0,1);
for k=1:N0
    carpeta_raiz(k,1) = cellstr(nivel1(k).name);
end

for k = 1:N0
    cd(carpeta_raiz{k,1})
    % Se determinan las carpetas en el nivel de clasificación por finalidad
    nivel2 = dir(fullfile(pwd,'/*'));
    N1 = length(nivel2(3:end));
    carpeta_subnivel = strings(N1,1);
    for i=1:N1
        carpeta_subnivel(i,1) = cellstr(nivel2(2+i).name);
    end
    for i = 1:N1
        cd(carpeta_subnivel{i,1})
        % Las carpetas con topics que se puedan leer en Matlab tienen el
        % strig característico '*Ensayo*'
        fprintf('\nProcesando %s.\n',pwd);
        Ensayos = dir(fullfile(pwd,'*Ensayo*'));
        [w0,w1]=size(Ensayos);
        if w0~=0
            Datos = strings(w0,1);
            for j=1:w0
                Datos(j,1) = cellstr(Ensayos(j).name);
            end
            for j=1:w0
                cd(Datos{j,1})
                save SREG.mat
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                Destino = pwd;
                x = strfind(Destino,'\')';
                title = [Destino(x(end)+1:end),'.mat'];
                % Se comprueba que el ensayo no se haya importado
                % anteriormente
                aux = strfind(archivos_reg,title);
                aux1 = find(~cellfun('isempty', aux));
                % Si no se ha registrado el ensayo con anterioridad, se
                % realiza el if
                if isempty(aux1)
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    %%%%% Topics de la versión firmware más reciente %%%%%%
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    fprintf('\tProcesando %s...\n',title);
                    %+-------------------------+
                    %|    Amm_Ammeter_value    |
                    %+-------------------------+
                    if exist('rostopic_Amm_Ammeter_value.txt','file')
                        fprintf('\t\tAmm_Ammeter_value ...\n');
                        A = importdata('rostopic_Amm_Ammeter_value.txt');
                        if ~isempty(A)
                            Amm_Ammeter_value = A.data;
                        end
                    end
                    %+---------------------------+
                    %|    Amm_Battery_voltaje    |
                    %+---------------------------+
                    if exist('rostopic_Amm_Battery_voltaje.txt','file')
                        fprintf('\t\tAmm_Battery_voltaje ...\n');
                        A = importdata('rostopic_Amm_Battery_voltaje.txt');
                        if ~isempty(A)
                            Amm_Battery_voltaje = A.data;
                        end
                    end
                    %+-----------------------+
                    %|    Autonomous_mode    |
                    %+-----------------------+
                    if exist('rostopic_Autonomous_mode.txt','file')
                        fprintf('\t\tAutonomous_mode ...\n');
                        A = importdata('rostopic_Autonomous_mode.txt');
                        if ~isempty(A)
                            Autonomous_mode = A.data;
                        end
                    end
                    %+-------------------------+
                    %|    Bat_Ammeter_value    |
                    %+-------------------------+
                    if exist('rostopic_Bat_Ammeter_value.txt','file')
                        fprintf('\t\tBat_Ammeter_value ...\n');
                        A = importdata('rostopic_Bat_Ammeter_value.txt');
                        if ~isempty(A)
                            Bat_Ammeter_value = A.data;
                        end
                    end
                    %+---------------------------+
                    %|    Bat_Battery_voltaje    |
                    %+---------------------------+
                    if exist('rostopic_Bat_Battery_voltaje.txt','file')
                        fprintf('\t\tBat_Battery_voltaje ...\n');
                        A = importdata('rostopic_Bat_Battery_voltaje.txt');
                        if ~isempty(A)
                            Bat_Battery_voltaje = A.data;
                        end
                    end
                    %+--------------------+
                    %|    Brake_enable    |
                    %+--------------------+
                    if exist('rostopic_Autonomous_mode.txt','file')
                        fprintf('\t\tBrake_enable ...\n');
                        A = importdata('rostopic_Autonomous_mode.txt');
                        if ~isempty(A)
                            Autonomous_mode = A.data;
                        end
                    end
                    %+-----------+
                    %|   Eje_x   |
                    %+-----------+
                    if exist('rostopic_Eje_x.txt','file')
                        fprintf('\t\tEje_x ...\n');
                        A = importdata('rostopic_Eje_x.txt');
                        if ~isempty(A)
                            Eje_x = A.data;
                        end
                    end
                    %+-----------+
                    %|   Eje_y   |
                    %+-----------+
                    if exist('rostopic_Eje_y.txt','file')
                        fprintf('\t\tEje_y ...\n');
                        A = importdata('rostopic_Eje_y.txt');
                        if ~isempty(A)
                            Eje_y = A.data;
                        end
                    end 
                    %+-----------+
                    %|   Eje_z   |
                    %+-----------+
                    if exist('rostopic_Eje_z.txt','file')
                        fprintf('\t\tEje_z ...\n');
                        A = importdata('rostopic_Eje_z.txt');
                        if ~isempty(A)
                            Eje_z = A.data;
                        end
                    end
                    %+-----------------+
                    %|   Enc_phidget   |
                    %+-----------------+
                    if exist('rostopic_Enc_phidget.txt','file')
                        fprintf('\t\tEnc_phidget ...\n');
                        A = importdata('rostopic_Enc_phidget.txt');
                        if ~isempty(A)
                            Enc_phidget = A.data;
                        end
                    end
                    %+-------------------+
                    %|   Enc_phidget_0   |
                    %+-------------------+
                    if exist('rostopic_Enc_phidget_0.txt','file')
                        fprintf('\t\tEnc_phidget_0 ...\n');
                        A = importdata('rostopic_Enc_phidget_0.txt');
                        if ~isempty(A)
                            Enc_phidget0 = A.data;
                        end
                    end
                    %+-------------------+
                    %|   Enc_phidget_1   |
                    %+-------------------+
                    if exist('rostopic_Enc_phidget_1.txt','file')
                        fprintf('\t\tEnc_phidget_1 ...\n');
                        A = importdata('rostopic_Enc_phidget_1.txt');
                        if ~isempty(A)
                            Enc_phidget1 = A.data;
                        end
                    end
                    %+-------------------+
                    %|   Enc_phidget_2   |
                    %+-------------------+
                    if exist('rostopic_Enc_phidget_2.txt','file')
                        fprintf('\t\tEnc_phidget_2 ...\n');
                        A = importdata('rostopic_Enc_phidget_2.txt');
                        if ~isempty(A)
                            Enc_phidget2 = A.data;
                        end
                    end
                    %+-------------------+
                    %|   Enc_phidget_3   |
                    %+-------------------+
                    if exist('rostopic_Enc_phidget_3.txt','file')
                        fprintf('\t\tEnc_phidget_3 ...\n');
                        A = importdata('rostopic_Enc_phidget_3.txt');
                        if ~isempty(A)
                            Enc_phidget3 = A.data;
                        end
                    end
                    %+-------------------+
                    %|   Enc_phidget_4   |
                    %+-------------------+
                    if exist('rostopic_Enc_phidget_4.txt','file')
                        fprintf('\t\tEnc_phidget_4 ...\n');
                        A = importdata('rostopic_Enc_phidget_4.txt');
                        if ~isempty(A)
                            Enc_phidget4 = A.data;
                        end
                    end
                    %+-------------------+
                    %|   Enc_phidget_5   |
                    %+-------------------+
                    if exist('rostopic_Enc_phidget_5.txt','file')
                        fprintf('\t\tEnc_phidget_5 ...\n');
                        A = importdata('rostopic_Enc_phidget_5.txt');
                        if ~isempty(A)
                            Enc_phidget5 = A.data;
                        end
                    end
                    %+-----------------+
                    %|   Encoder_Abs   |
                    %+-----------------+
                    if exist('rostopic_Encoder_Abs.txt','file')
                        fprintf('\t\tEncoder_Abs ...\n');
                        A = importdata('rostopic_Encoder_Abs.txt');
                        if ~isempty(A)
                            Encoder_Abs = A.data;
                        end
                    end
                    %+----------+
                    %|   Odom   |
                    %+----------+
                    if exist('rostopic_Odom.txt','file')
                        fprintf('\t\tOdom ...\n');
                        A = importdata('rostopic_Odom.txt');
                        if ~isempty(A)
                            Odom = A.data;
                        end
                    end
                    %+-----------------+
                    %|   OL_Steering   |
                    %+-----------------+
                    if exist('rostopic_OL_Steering.txt','file')
                        fprintf('\t\tOL_Steering ...\n');
                        A = importdata('rostopic_OL_Steering.txt');
                        if ~isempty(A)
                            OL_Steering = A.data;
                        end
                    end
                    %+-----------------+
                    %|   OL_Throttle   |
                    %+-----------------+
                    if exist('rostopic_OL_Throttle.txt','file')
                        fprintf('\t\tOL_Throttle ...\n');
                        A = importdata('rostopic_OL_Throttle.txt');
                        if ~isempty(A)
                            OL_Throttle = A.data;
                        end
                    end
                    %+---------------------+
                    %|   Speedcruise_adc   |
                    %+---------------------+
                    if exist('rostopic_Speedcruise_adc.txt','file')
                        fprintf('\t\tSpeedcruise_adc ...\n');
                        A = importdata('rostopic_Speedcruise_adc.txt');
                        if ~isempty(A)
                            Speedcruise_adc = A.data;
                        end
                    end
                    %+--------------------------------+
                    %|   Speedcruise_control_signal   |
                    %+--------------------------------+
                    if exist('rostopic_Speedcruise_control_signal.txt','file')
                        fprintf('\t\tSpeedcruise_control_signal ...\n');
                        A = importdata('rostopic_Speedcruise_control_signal.txt');
                        if ~isempty(A)
                            Speedcruise_control_signal = A.data;
                        end
                    end
                    %+--------------------------+
                    %|   Speedcruise_encoders   |
                    %+--------------------------+
                    if exist('rostopic_Speedcruise_encoders.txt','file')
                        fprintf('\t\tSpeedcruise_encoders ...\n');
                        A = importdata('rostopic_Speedcruise_encoders.txt');
                        if ~isempty(A)
                            Speedcruise_encoders = A.data;
                        end
                    end
                    %+---------------+
                    %|   Steer_adc   |
                    %+---------------+
                    if exist('rostopic_Steer_adc.txt','file')
                        fprintf('\t\tSteer_adc ...\n');
                        A = importdata('rostopic_Steer_adc.txt');
                        if ~isempty(A)
                            Steer_adc = A.data;
                        end
                    end
                    %+--------------------------+
                    %|   Steer_control_signal   |
                    %+--------------------------+
                    if exist('rostopic_Steer_control_signal.txt','file')
                        fprintf('\t\tSteer_control_signal ...\n');
                        A = importdata('rostopic_Steer_control_signal.txt');
                        if ~isempty(A)
                            Steer_control_signal = A.data;
                        end
                    end
                    %+--------------------+
                    %|   Steer_encoders   |
                    %+--------------------+
                    if exist('rostopic_Steer_encoders.txt','file')
                        fprintf('\t\tSteer_encoders ...\n');
                        A = importdata('rostopic_Steer_encoders.txt');
                        if ~isempty(A)
                            Steer_encoders = A.data;
                        end
                    end
                    %+--------+
                    %|   Tf   |
                    %+--------+
                    if exist('rostopic_Tf.txt','file')
                        fprintf('\t\tTf ...\n');
                        A = importdata('rostopic_Tf.txt');
                        if ~isempty(A)
                            Tf = A.data;
                        end
                    end
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    %%%%%% Topics de versiones antiguas del firmware %%%%%%
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    %+---------+
                    %|   ADC   |
                    %+---------+
                    if exist('rostopic_ADC.txt','file') ~= 0
                        fprintf('\t\tADC ...\n');
                        A = importdata('rostopic_ADC.txt');
                        if ~isempty(A)
                            ADC = A.data;
                        end
                    end
                    %+--------------------+
                    %|   Control_signal   |
                    %+--------------------+
                    if exist('rostopic_Control_signal.txt','file')
                        fprintf('\t\tControl_signal ...\n');
                        A = importdata('rostopic_Control_signal.txt');
                        if ~isempty(A)
                            Control_signal = A.data;
                        end
                    end
                    %+--------------+
                    %|   Encoders   |
                    %+--------------+
                    if exist('rostopic_Encoders.txt','file')
                        fprintf('\t\tEncoders ...\n');
                        A = importdata('rostopic_Encoders.txt');
                        if ~isempty(A)
                            Encoders = A.data;
                        end
                    end
                    %+----------------------+
                    %|     Labview Data     |
                    %+----------------------+
                    if exist('Labview_Data.txt','file')
                        fprintf('\t\tLabview Data ...\n');
                        A = importdata('Labview_Data.txt');
                        if ~isempty(A)
                            Labview = A.data;
                        end
                    end
                    cd(Origen);
                    clearvars -except -regexp ^Steer_ ^Speedcruise ^Amm_ ...
                        ^Autonomous_ ^Bat_ ^Eje_ ^Enc_ ^Encoder_Abs ...
                        ^Odom ^OL_ ^Tf ^Labview ^Encoders ^Control_s ...
                        ^ADC ^Destino ^title
                    
                    cd('ECARM MATLAB')
                    save(title)
                    cd(Destino);
                    fprintf('\tRegistrado \n');
                else
                    fprintf('\t%s ya está registrado\n',title);
                end
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                clear variables
                load SREG.mat
                delete SREG.mat
                cd ..
            end
        else
            fprintf('\tNo contiene datos de ensayos para matlab\n');
        end
        cd ..
    end
    cd ..
end
clear variables