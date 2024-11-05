 function [Serial_Port,com_stab] = Connect_AAL_BAND()

Serial_Port = 0;
com_port_write_error = 0;
com_port_open_error = 0;

%% With Serial Port

%COM_Port = getAvailableComPort(); % Looking for available COM ports
COM_Port = {"COM127","COM129"}; % Looking for available COM ports  26E6
[~,~]=size(COM_Port);
% [m,~] = size(COM_Port);%%将串�?�的行数返给m，如�?边~是第二个�?��?则将列返给其
com_stab = 0;
i = 1;
while (com_stab == 0)
    
    Serial_Port = serial(COM_Port{i},'BaudRate',250000, 'DataBits',8,'Timeout',3);%%建立串�?�对象函数：obj=seril（’port’,’property name’,propertyvalue……），其中主�?的属性有：baudrate（波特率），databits（数�?��?），parity（校验方�?），stopbits（终止�?）等，�?�以在�?始化时进行赋值或者使�"�set函数。
    pause(0.5)%%延迟秒数
    try
        fopen(Serial_Port);
    catch 
        com_port_open_error = 1;
    end
    if(com_port_open_error == 0)
        pause(0.5)
        try
            fwrite(Serial_Port,'0');%%当matlab通信数�?�采用二进制格�?时，读写串�?�设备的命令为fread()和fwrite()；当通行数�?�采用文本（ASCII）格�?时，读写串�?�设备的命令为fscanf（）和fprintf（）
        catch
            com_port_write_error = 1;
        end
        if(com_port_write_error == 0)
            Recieve_bit = fscanf(Serial_Port);
            TF = isempty(Recieve_bit);
            if (TF==0)
                if (Recieve_bit(1)=='1')
                    Serial_COM = COM_Port(i);
                    com_stab = 1;
                else
                    delete(Serial_COM);
%                     delete(Serial_Port);
                end
            end
        else
            com_port_write_error = 0;
        end
    else
        com_port_open_error = 0;
    end
%     if(i == m)
%        break;
%     end
    
    i = i + 1;
end
