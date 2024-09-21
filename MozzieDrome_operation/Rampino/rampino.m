mpino %RAMPINO 221216.1 Include pinMode() to set PWM frequency
%RAMPINO 221212.1 (aka V1.3) Include support for slow serial usb opening and I2C sensors
%RAMPINO V1.2 Nov 2022--Added DOT STAR LED SUPPORT
%History V1.1 --Added Stteper limit switch polarity set
classdef rampino < handle
    % This class creates an "rampino" object to control
    % Ramps 1.4...1.6+ shield board on Arduino Mega from Matlab.  
    % Derived from old arduino io code by Giampiero Campa 
    % by AIC at Columbia university, Richard Hormigo 2021
    % Contact Rick Hormigo for last released code
    %
    %Syntax
    %r=rampino(Port,Validation)
    %   Port is the serial port used by the Mega Board such as 'COM3'
    %   Validation does validate the parameters used in the Rampino control
    %   can be true or false. With false validation is skip for faster communications.
    %r=rampino(Port) 
    %   Same than above, but Validation is true.
    
    properties (SetAccess=private,GetAccess=private)
        aser   % Serial Connection
        aserOK=false % Serial link flag ok
        pins   % Pin Status Vector  (-1 for unassigned, 0 for input, 1 for output, 2 for input_pullUp)
        srvs   % Servo Status Vector(0=Detached, 1 Attached)
        dcv    % DC Motors Velocity status (+/- 100%)
        dcs    % DC Motors Status
        sspd   % Stepper Motors Speed Status
               %    Field 1 constant speed in (0-255 RPMs) 
               %    Field 2 speed motion (0 is CONSTANT 0, or 1 is LINEAR
               %    Field 3 Speed Linear Mode Acceleration 0 to 65535 steps/s^2
               %    Field 4 Speed Linear Mode Deceleration 0 to 65535 steps/s^2
        steps  % Stepper Motors Status 
               %    Field 1- Hardware Mode (0 is detached, 1 is attached software, 2 is attached hardware)
               %    Field 2- Steps Remaining.
        encs   % Encoders Status (0=Detached, 1 Attached)
        sktc   % Motor Server Running on the rampino Board  
    end
    
    properties (Hidden=true)
        chkp = true;   % Checks parameters before every operation
    end
    
    methods
        % constructor, connects to the board and creates a rampino object
        function r=rampino(comPort,chkp) 
            % check nargin
            if nargin<1
                disp('No Port selected');
                disp('Use a COM port, e.g. ''COM3'' as input argument to connect to Ramps'); 
                return;
            elseif nargin==1
                r.chkp=true;
            else % check parameter bypass 
                if ~islogical(chkp)
                    error('The second argument is to validate parameters across rampino, if used, must be true (default) or false for faster execution');
                else
                    r.chkp=chkp;
                end
            end
     
            % check port
            if ~ischar(comPort)
                error('The first argument is the serial port and must be a string, e.g. ''COM3'' ');
            end     
            % check if port is available
            if ~any(strcmp(serialportlist("available"),comPort))
                disp([num2str(comPort), ' port is not available. If you are already connected,' ]);
                disp('delete or clear the rampino object before trying again.');
                disp('Also the port may not exist, or be taken by another terminal program');
                disp([serialportlist("available"), ' are listed available. Try one of these.' ]);
                return;
            end
            % open port
            try
                r.aser=serialport(comPort,115200); %tried up to 250000 Bauds that is multiple of 16M and available at Arduino IDE Monitor
                %Serial in the arduino depending on computer and driver may take sevela seconds to open, as the arduino is reset automatically at this opening
                %If we send data before is fully open, the driver will get stuck, and may need several seconds to clear.
                %So rampino will send an initial 'R' cha to indicate serial port is up and we can proceed.
                fprintf("Attempting connection...");
                while r.aser.NumBytesAvailable==0 %wait here until we get a string with version (8 chars) 
                    pause(0.5)
                    fprintf('.');
                end
                fprintf("\nDetected server revision %s\n", read(r.aser,8,'char'));
                flush(r.aser); %OK so char came, flush and move on now.
                configureTerminator(r.aser,"CR/LF"); %used only with writeline and readline
            catch ME
                disp(ME.message)
                delete(r);
                error(['Could not open Ramps at: ' comPort]);
            end
            %Now Validate is a rampino server
            write(r.aser,[57 57],'char');
            r.sktc=read(r.aser,1,'char');  %ASCII code of sketch 'R' Rampino
            % exit if there was no answer
            if isempty(r.sktc)
                delete(r);
                error('Sketch connection unsuccessful, please make sure that the Arduino Mega board is powered on, running the Rampino sketch, and connected to the called serial port. You might also try to unplug and re-plug the USB cable before attempting a reconnection.');
            end
            % check returned value
            if r.sktc==double('R') %R is the Rampino Sketch 
                disp('Standard Rampino server detected !');
            else
                delete(r);
                error('Unknown sketch response from Arduino Mega. Please make sure the Rampino sketch is uploaded and running on the board');
            end
            % set validation flag          
            r.aserOK =true;
            % initialize pin vector (-1 is unassigned, 0 is input, 1 is output, 2 is input with pullup)
            r.pins=-ones(1,69);
            % initialize servo vector (0 is detached, 1 is attached)
            r.srvs=zeros(1,69);
            % initialize encoder vector (0 is detached, 1 is attached)
            r.encs=zeros(1,3);
            % initialize DC motor velocity vector (-100% to 100% pwm velocity)
            r.dcv=zeros(1,5);
            % initialize DC motor vector (0 is detached, 1 is attached, 2 is Attached flip direction)
            r.dcs=zeros(1,5);
            % initialize stepper speed and profile status vector 
               %    Field 1 constant speed in 0-255 RPMs 
               %    Field 2 speed motion (0 is CONSTANT 0, or 1 is LINEAR
               %    Field 3 Speed Linear Mode Acceleration 0 to 65535 steps/s^2
               %    Field 4 Speed Linear Mode Deceleration 0 to 65535 steps/s^2
            r.sspd=repmat([0 0 1000 1000],5,1);
            % initialize stepper vector status  
                %Field 2- Hardware Mode (0 is detached, 1 is attached software, 2 is attached hardware )
                %Field 1- Steps Remaining.
  %          r.steps=zeros(5,2);
            % notify successful installation            
            disp('Rampino successfully connected !');       
        end % rampino constructor   
        % destructor, deletes the object
        function delete(r)
            % Use delete(r) or r.delete to delete the rampino object
           if r.aserOK 
                try % trying to leave it in a known unharmful state                   
                    for i=2:69
                        r.pinMode(i,'input');
                    end
                catch ME
                    % disp but proceed anyway
                    disp(ME.message);
                    disp('Proceeding to deletion anyway');
                end    
           end
            % if it's an object delete it
            if isobject(r.aser)
                delete(r.aser);
            end
        end % delete
        
        % disp, displays the object
        function disp(r)  
            % disp(r) or r.disp, displays the rampino object properties
            % The first and only argument is the rampino object, there is no
            % output, but the basic information and properties of the rampino
            % object are displayed on the screen
            % This function is called automatically after the rampino object is created
            % and no semicolon is used after the class declaration. 
            
            if isvalid(r) 
                if isa(r.aser,'internal.Serialport') && isvalid(r.aser)
                    disp(['<a href="matlab:help rampino">Rampino</a> object connected to port ', num2str(r.aser.Port)]);
                    if r.sktc==double('R')
                        disp('Rampino over Mega I/O, with Poly Tones, Servos, Encoders, DC Motor (on Stepper slots),  and A4988 type Steppers running on the board');
                        disp(' ');                        
                        r.pinMode
                        disp('Pin IO Methods: <a href="matlab:help pinMode">pinMode</a> <a href="matlab:help digitalRead">digitalRead</a> <a href="matlab:help digitalWrite">digitalWrite</a> <a href="matlab:help analogRead">analogRead</a> <a href="matlab:help analogWrite">analogWrite</a> <a href="matlab:help analogReference">analogReference</a>');
                        disp(' ');
                        disp('Polyphonic tone player: <a href="matlab:help tonePlay">tonePlay</a>');
                        disp(' ');
                        r.servoStatus
                        disp('Servo Methods:  <a href="matlab:help servoAttach">servoAttach</a> <a href="matlab:help servoDetach">servoDetach</a> <a href="matlab:help servoStatus">servoStatus</a> <a href="matlab:help servoRead">servoRead</a> <a href="matlab:help servoWrite">servoWrite</a>');
                        disp(' ');
                        r.encoderStatus
                        disp('Encoder Methods: <a href="matlab:help encoderStatus">encoderStatus</a> <a href="matlab:help encoderAttach">encoderAttach</a> <a href="matlab:help encoderDetach">encoderDetach</a> <a href="matlab:help encoderRead">encoderRead</a> <a href="matlab:help encoderReset">encoderReset</a> <a href="matlab:help encoderDebounce">encoderDebounce</a>');
                        disp(' ');
                        r.DCMotorStatus
                        r.DCMotorVelocity
                        disp('DC Motor Methods: <a href="matlab:help DCMotorAttach">DCMotorAttach</a> <a href="matlab:help DCMotorDetach">DCMotorDetach</a> <a href="matlab:help DCMotorStatus">DCMotorStatus</a> <a href="matlab:help DCMotorVelocity">DCMotorVelocity</a>');
                        disp(' ');
                        r.stepperStatus
                        r.stepperSpeed
                        disp('Stepper Motor Methods: <a href="matlab:help stepperAttach">stepperAttach</a> <a href="matlab:help stepperDetach">stepperDetach</a> <a href="matlab:help stepperStatus">stepperStatus</a> <a href="matlab:help stepperSpeed">stepperSpeed</a> <a href="matlab:help stepperStep">stepperStep</a>');
                        disp(' ');
                        disp(['Serial port and other Methods:  <a href="matlab:help serialportlist">serialportlist</a> <a href="matlab:help '  inputname(1) '.serialport">' inputname(1) '.serialport</a> <a href="matlab:help '  inputname(1) '.flush">'  inputname(1) '.flush</a> <a href="matlab:help roundTrip">roundTrip</a>']);
                   else
                        disp('This is awkward... no valid board sketch detected');
                   end
                    disp(' ');
                else
                    disp('<a href="matlab:help rampino">rampino</a> object is invalid, need a valid serial port');
                    disp('Please delete the rampino object');
                    disp(' ');
                end
            else
                disp('Invalid <a href="matlab:help rampino">rampino</a> object');
                disp('Please clear the object and instantiate another one');
                disp(' ');
            end
        end % disp 
        
        % serialport, returns the serial port
        function str=serialport(r)
            % serialport(r) (or r.serialport), returns the name of the serial port
            % and roundTrip time for 2 bytes TX plus one RX 
            % The first and only argument is the rampino object, the output
            % is a string containing the name of the serial port to which
            % the rampino board is connected (e.g. 'COM9' or '/dev/ttyS101')
            % and timing. The strings 'Link with Rampino is invalid' or
            % 'broken' if there is a serial communication error.
          
            if isvalid(r.aser)%Check if we are still connected to rampino and report port and timming             
                if r.roundTrip(55)~=55
                    str='Link with Rampino is broken';
                else
                    pause(0.1);
                    tic;r.roundTrip(55);tval=toc;
                    str= strcat('Serial port is:  ',num2str(r.aser.Port), '. Roundtrip for 2 bytes TX and 1 byte RX is:  ', num2str(tval,'%.6f'), ' Seconds');
                end
            else
                str='Link with Rampino is invalid';
            end
        end  % serialport
        
        % flush, clears the pc's serial port buffer
        function val=flush(r)
            % val=flush(r) (or val=r.flush) reads all the bytes available 
            % (if any) in the computer's serial port buffer, therefore 
            % clearing said buffer.
            % The first and only argument is the rampino object, the 
            % output is a vector of bytes that were still in the buffer.
            % The value '-1' is returned if the buffer was already empty.
            
            val=-1;
            if r.aser.NumBytesAvailable>0
                val=read(r.aser,r.aser.NumBytesAvailable,'char');
            end
        end  % flush

        % round trip
        function val=roundTrip(r,byte)
            % roundTrip(r,byte); sends something to the rampino and back
            % The first argument, r, is the rampino object.
            % The second argument, byte, is any integer from 0 to 255.
            % The output is the same byte, which was received from the
            % rampino and sent back along the serial connection unchanged.
            %
            % This is used by the serialport(r) method to test the serial
            % communications  with rampino. Also can be used as a simple
            % example to add additional custom commands in the rampino environment  
            % This basic echo function is in the server sketch file handled
            % as sate (case 400:)
            %
            % Examples:
            % roundTrip(r,48); % sends '48' to the rampino and back.
            % r.roundTrip(53); % sends '53' to the rampino and back.
            
            %%%%%%%%%%%%%%%%%%%%%%%%% ARGUMENT CHECKING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % check arguments if r.chkp is true
            if r.chkp
                % check nargin
                if nargin~=2
                    error('Function must have one argument');
                end
                % check argument (must be a byte)
                errstr=rampino.checknum(byte,'byte',0:255);
                if ~isempty(errstr), error(errstr); end
            end
            %%%%%%%%%%%%%%%%%%%%%%%%% SEND ARGUMENT ALONG %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            write(r.aser,[88 byte],'uint8'); %TX
            val=read(r.aser,1,'uint8');      %RX   
        end % roundtrip

        % pin mode, changes pin mode
        function pinMode(r,pin,str)
            % pinMode(r,pin,str); reads or sets the I/O mode of a digital pin.
            % The first argument, r, is the rampino object.
            % The second argument, pin, is the number of the digital pin (2 to 69).
            % The third argument, str, is a string that can be 'input','output',
            % or 'pullUp'. Also I, O, or P, will work.
            % Added in REV 221216.1 there is also 1 to 5 argument to set
            % the timer dividers in PWM pins so some special PWM frequencies con be set
            % This works for pins 3, and 5 to 12 OR pins 4 and 13 as follows:
            % str=1 31372.55 Hz OR 62500 Hz
            % str=2 3921.16 Hz / 7812.5 Hz
            % str=3 490.20 Hz / 976.56 Hz  (Default)
            % str=4 122.55 Hz / 244.14 Hz
            % str=5 30.61 Hz / 61.04 Hz 
            % Called as pinMode(r,pin) it returns the mode of the digital pin, 
            % called as pinMode(r), it prints the mode of all the digital pins. 
            % Note that rampino is based in Ramps 1.6 over Arduino MEGA 2560
            % where all but pins 7 and 22 are available.
            % Also note that digital pins 54 to 63 are the same than analog 0 to 15
            % The RampinoConnector2ArduinoMegaPinMap.pdf document shows
            % all Mega pins to Ramps connectors mapping
            %
            % Examples:
            % pinMode(r,11,'output') % sets digital pin #11 as output
            % pinMode(r,10,'i')      % sets digital pin #10 as input
            % pinMode(r,5,'P')       % sets digital pin #5 as input with pullup
            % r.pinMode(10,'input')  % same as pinMode(r,10,'input')
            % val=pinMode(r,10);     % returns the status of digital pin #10
            % pinMode(r,5);          % prints the status of digital pin #5
            % pinMode(r);            % prints the status of all pins
            
            %%%%%%%%%%%%%%%%%%%%%%%%% ARGUMENT CHECKING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % check arguments if r.chkp is true
            if r.chkp              
                % check nargin
                if nargin>3
                    error('This function cannot have more than 3 arguments, object, pin and str');
                end
                % if pin argument is there check it
                if nargin>1
                    errstr=rampino.checknum(pin,'pin number',2:69);
                    if ~isempty(errstr), error(errstr); end
                end
                % if str argument is there check it
                if nargin>2
                    errstr=rampino.checkstr(str,'pin mode',{'input','output','pullUp','I','O','P'});
                    if str>0 && str<6, errstr=''; end %Special PWM freq case 1 to 5
                    if ~isempty(errstr), error(errstr); end
                end                
            end 
            % perform pin mode
            if nargin==3
                %%%%%%%%%%%%%%%%%%%%%%%%% CHANGE PIN MODE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % assign value
                if lower(str(1))=='p'  %input_pullup
                    val=2; 
                elseif lower(str(1))=='o' %output
                    val=1;
                else  %input
                    val=0;   
                end
                if str>0 && str<6, val=str+2; end %Calculate ASCII value
                % send mode, pin and value
                write(r.aser,[48 97+pin 48+val],'char');
                % store r.pins locally with 0 for input, 1 for output, 2 for input_pullUp
                % 3 for timer divider 1, to 7 for timer divider 5
                % Initially r.pins = -1 (Unassigned) that practically is as Input  
                r.pins(pin)=val; 
            elseif nargin==2
                % print pin mode for the requested pin
                mode={'UNASSIGNED','set as INPUT','set as OUTPUT','set as INPUT_PULLUP', 'set with Divider 1',...
                    'set with Divider 2', 'set with Divider 3', 'set with Divider 4', 'set with Divider 5' };
                disp(['Digital Pin ' num2str(pin) ' is currently ' mode{2+r.pins(pin)}]);
            else
                % print pin mode for each pin   
                mode={'UNASSIGNED','set as INPUT','set as OUTPUT','set as INPUT_PULLUP', 'set with Divider 1',...
                    'set with Divider 2', 'set with Divider 3', 'set with Divider 4', 'set with Divider 5' };
                for i=2:69
                    disp(['Digital Pin ' num2str(i,'%02d') ' is currently ' mode{2+r.pins(i)}]);
                end
            end
        end % pinmode
        
        % digital read
        function val=digitalRead(r,pin)           
            % val=digitalRead(r,pin); performs digital input read on a given rampino pin.
            % The first argument, r, is the rampino object.
            % The second argument, pin, is the number of the digital pin (2 to 69)
            % where the digital input needs to be performed.
            % returns 0 or 1, for Low or High 
            % Note that rampino is based in Ramps 1.6 over Arduino MEGA 2560
            % where all but pins 7 and 22 are available.
            % Also note that digital pins 54 to 63 are the same than analog 0 to 15
            % The RampinoConnector2ArduinoMegaPinMap.pdf document shows
            % all Mega pins to Reamps connectors mapping   
            %
            % Examples:
            % val=digitalRead(r,4); % reads pin #4
            % val=r.digitalRead(4); % just as above (reads pin #4)

            %%%%%%%%%%%%%%%%%%%%%%%%% ARGUMENT CHECKING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % check arguments if r.chkp is true
            if r.chkp
                % check nargin
                if nargin~=2
                    error('Function must have the "pin" argument');
                end
                % check pin
                errstr=rampino.checknum(pin,'pin number',2:69);
                if ~isempty(errstr), error(errstr); end
            end         
            %%%%%%%%%%%%%%%%%%%%%%%%% PERFORM DIGITAL INPUT %%%%%%%%%%%%%%%%%%%%%%%%%%%                
            % send mode and pin
            write(r.aser,[49 97+pin],'char');
            % get value
            val=read(r.aser,1,'uint8')-48;         
        end % digitalread
        
        % digital write
        function digitalWrite(r,pin,val)
            % digitalWrite(r,pin,val); performs digital output on a given pin.
            % The first argument, r, is the rampino object.
            % The second argument, pin, is the number of the digital pin 
            % (2 to 69) where the digital output value needs to be written.
            % The third argument, val, is the output value (either 0 or 1).
            % Note that rampino is based in Ramps 1.6 over Arduino MEGA 2560
            % where all but pins 7 and 22 are available.
            % Also note that digital pins 54 to 63 are the same than analog 0 to 15
            % The RampinoConnector2ArduinoMegaPinMap.pdf document shows
            % all Mega pins to Reamps connectors mapping   
            %
            % Examples:
            % digitalWrite(r,13,1); % sets pin #13 high
            % digitalWrite(r,13,0); % sets pin #13 low
            % r.digitalWrite(13,0); % just as above (sets pin #13 to low)
            %
            
            %%%%%%%%%%%%%%%%%%%%%%%%% ARGUMENT CHECKING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
            % check arguments if r.chkp is true
            if r.chkp       
                % check nargin
                if nargin~=3
                    error('Function must have the "pin" and "val" arguments');
                end       
                % check pin
                errstr=rampino.checknum(pin,'pin number',2:69);
                if ~isempty(errstr), error(errstr); end
                % check val
                errstr=rampino.checknum(val,'value',0:1);
                if ~isempty(errstr), error(errstr); end            
                % get object name
                if isempty(inputname(1))
                    name='object'; 
                else
                    name=inputname(1);
                end   
                % pin should be configured as output
                if r.pins(pin)~=1
                    warning('MATLAB:rampino:digitalWrite',['If digital pin ' num2str(pin) ' is set as input, digital output takes place only after using ' name' '.pinMode(' num2str(pin) ',''output''); ']);
                end  
            end  
            %%%%%%%%%%%%%%%%%%%%%%%%% PERFORM DIGITAL OUTPUT %%%%%%%%%%%%%%%%%%%%%%%%%%       
            % send mode, pin and value
            write(r.aser,[50 97+pin 48+val],'char');
        end % digitalwrite
        
        % analog read
        function val=analogRead(r,pin)
            % val=analogRead(r,pin); Performs analog input on a given rampino pin.
            % The first argument, r, is the rampino object. The second argument, 
            % pin, is the number of the analog input pin (0 to 15) from which the 
            % analog value needs to be read. The returned value, val, ranges from 
            % 0 to 1023, with 0 corresponding to an input voltage of 0 volts,
            % and 1023 to a reference value that is by default 5 volts (this reference can
            % be set up by the analogReference function). Therefore, assuming a range
            % from 0 to 5 V the resolution is .0049 volts (4.9 mV) per unit.
            % For the ramps board that uses Arduino Mega the analog input pins 0 to 15 are also
            % the digital pins from 54 to 69
            % Performing analog input does not affect the digital state of the pin.
            % The RampinoConnector2ArduinoMegaPinMap.pdf document shows
            % all Mega pins to Reamps connectors mapping. Note that the motor drivers (if used)
            % take several analog pins in digital mode:
            % A3, A4 A5, A10, A13 to A15 are individually available
            % A0,A1,A2,A6,A7 and A8, go directly to motor drivers 
            % A9,A11,and A12, are taken for microstepping setting (only if jumpers set)
            %
            % Examples:
            % val=analogRead(r,3); % reads analog input pin A3
            % val=r.analogRead(3); % just as above, reads analog input pin A3
            %
                    
            %%%%%%%%%%%%%%%%%%%%%%%%% ARGUMENT CHECKING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % check arguments if r.chkp is true
            if r.chkp
                % check nargin
                if nargin~=2
                    error('Function must have the "pin" argument');
                end
                % check pin
                errstr=rampino.checknum(pin,'analog input pin number',0:15);
                if ~isempty(errstr), error(errstr); end
            end 
            %%%%%%%%%%%%%%%%%%%%%%%%% PERFORM ANALOG INPUT %%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % send mode and pin
            write(r.aser,[51 97+pin],'char');
            % get value
            val=read(r.aser,1,'uint16');
        end % analogread
        
        % function analog write
        function analogWrite(r,pin,val)
            % analogWrite(r,pin,val); Performs "analog" PWM based output on a given pin.
            % The first argument, r, is the rampino object. The second argument, 
            % pin, is the number of the DIGITAL pin where the analog/PWM output 
            % needs to be performed. Allowed the Mega board used by rampino
            % are 2 to 13 and 44 to 46.
            % The second argument, val, is the value from 0 to 255 for the level of
            % analog output that translates in a pwm dutycycle=(value/255)*100 (%).
            % The RampinoConnector2ArduinoMegaPinMap.pdf document shows
            % all Mega pins to Reamps connectors mapping.
            %
            % Examples:
            % analogWrite(r,11,90); % sets pin #11 to 90/255= 35% duty
            % r.analogWrite(3,10); % sets pin #3 to 10/255= 4% duty
            %
            % NOTE: PWM signals are a simpler replacement to real analog outputs, 
            % often used for LEDs, solenoids, and motors. Also PWM can be
            % converted to a real analog value by passing thru a a low pass RC filter.
            % See more info at the arduino web site analogWrite() reference. 
            
            %%%%%%%%%%%%%%%%%%%%%%%%% ARGUMENT CHECKING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % check arguments if r.chkp is true
            if r.chkp 
                % check nargin
                if nargin~=3
                    error('Function must have the "pin" and "val" arguments');
                end
                % check pin
                errstr=rampino.checknum(pin,'pwm pin number',[2:13 44:46]);
                if ~isempty(errstr), error(errstr); end
                % check val
                errstr=rampino.checknum(val,'analog output level',0:255);
                if ~isempty(errstr), error(errstr); end
            end          
            %%%%%%%%%%%%%%%%%%%%%%%%% PERFORM ANALOG OUTPUT %%%%%%%%%%%%%%%%%%%%%%%%%%%
            % send mode, pin and value
            write(r.aser,[52 97+pin val],'char');  
        end % analogwrite
        
        % function analog reference
        function analogReference(r,str)
            % analogReference(r,str); Changes voltage reference on analog input pins
            % The first argument, r, is the rampino object. The second argument, 
            % str, is one of these strings: 'default', 'internal' or 'external'. 
            % This sets the reference voltage used at the top of the input ranges.
            %
            % Examples:
            % analogReference(r,'default'); % sets default reference 5V
            % analogReference(r,'internal'); % sets internal reference 1.1V (Mega)
            % analogReference(r,'external'); % sets external reference
            % r.analogReference('external'); % just as above (sets external reference)
            %
            %NOTE: The Ramps board has not connector for External Reference, 
            % but if needed still could be used directly on the Mega.
            
            %%%%%%%%%%%%%%%%%%%%%%%%% ARGUMENT CHECKING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % check arguments if r.chkp is true
            if r.chkp
                % check nargin
                if nargin~=2
                    error('Function must have the "reference" argument');
                end
                % check val
                errstr=rampino.checkstr(str,'reference',{'default','internal','external'});
                if ~isempty(errstr), error(errstr); end      
            end            
            %%%%%%%%%%%%%%%%%%%% CHANGE ANALOG INPUT REFERENCE %%%%%%%%%%%%%%%%%%%%%%%%%       
            if lower(str(1))=='e'
                num=2;
            elseif lower(str(1))=='i'
                num=1;
            else
                num=0;
            end              
            % send mode, pin and value
            write(r.aser,[82 48+num],'char');     
        end % analogreference
        
         % servo status
        function val=servoStatus(r,pin)     
            % servoStatus(r,pin); Reads if a servo is attached o detached.
            % The first argument, r, is the rampino object. The second argument, 
            % pin, is the number of the pin where the servo is attached 2 to 11
            % It can also be used the alias 'S1' to 'S4' to match the Ramps servo connectors.
            % See servoAttach for more information on pin limitations.
            % The function returns 0 for detached, or 1 for attached
            % if the output is not used, the strings 'DETACHED' or ;ATTACHED' are returned
            %
            % Examples:
            % val=servoStatus(r,10); % return the status of servo on pin #10
            % servoStatus(r,10); % prints the status of servo on pin #10
            % servoStatus(r); % prints the status of all servos
            % r.servoStatus; % prints the status of all servos
            
            %%%%%%%%%%%%%%%%%%%%%%%%% ARGUMENT CHECKING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%      
            % check nargin if r.chkp is true 
            if r.chkp         
                % check input argument number
                if nargin>2
                    error('Function cannot have more than one argument (servo number) beyond the object name');
                end
            end
            % with no arguments calls itself recursively for all servos
            if nargin==1
                if nargout>0
                    val=zeros(11,1);
                    for i=2:11
                        val(i)=r.servoStatus(i);
                    end
                    return
                else
                    for i=2:11
                        r.servoStatus(i);
                    end
                    return
                end
            end
            % check servo number if r.chkp is true
            if r.chkp
                errstr1=rampino.checknum(pin,'servo number',2:11);
                errstr2=rampino.checkstr(pin,'servo number',{'S1','S2','S3','S4'});
                if ~isempty(errstr1)&&~isempty(errstr2)
                    error([errstr1 ' OR ' errstr2])    
                end   
            end 
            %If Alias, convert to number
            rampsPServos=[11,6,5,4];
            rampsTServos={'S1','S2','S3','S4'};
            if(ischar(pin))
                pin=rampsPServos((find(contains({'s1','s2','s3','s4'},lower(pin)))));
            end
            %%%%%%%%%%%%%%%%%%%%%%%%% ASK SERVO STATUS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % send command and pin
            write(r.aser,[53 97+pin],'char');
            % check answer
            val=read(r.aser,1,'uint8');       
            % updates the servo state vector
            r.srvs(pin)=val;
            if nargout==0
                str={'DETACHED','ATTACHED'};
                if isempty(find(rampsPServos==pin,1)) %if no alias
                    disp(['Servo ' num2str(pin) ' is ' str{1+val}]);          
                else % has alias
                    disp(['Servo ' num2str(pin) ' (' rampsTServos{find(rampsPServos==pin,1)} ') is ' str{1+val}]);
                end
                clear val
                return
            end       
        end % servostatus
        
        % servo attach
        function servoAttach(r,pin)            
            % servoAttach(r,pin); or r.servoAttach(pin); attaches a servo to a pin.
            % The first argument, r, is the rampino object. The second argument, 
            % pin, is the number of the pwm pin where the servo must be attached.
            % It can also be used the alias 'S1' to 'S4' to match the Ramps servo connectors.
            % A servo has three pins: a central red one which goes to +5V,
            % a black or brown one which goes to ground, and a white or orange one 
            % which must be connected to the attached pin
            %
            % The mega board with rampino supports 10 Servos at pins 2 to 11,
            % but attaching any servo interferes (as they share timer 1) with analogWrite
            % at pin 11 and 12, that become ON with value of 255, else OFF.
            % But still 11 is ok for servo. The modified servo.h library that comes along
            % features to recover these two pins for PWM use, once all serors are detached.
            % The Ramps board on Mega has servo connectors 1 to 4  wired to pins 11, 6, 5, and 4 
            % respectivelly, that also support PWM. 
            % NOTE:the Arduino Servo.h library (used by this rampino server)
            % does NOT need PWM capable pins to control servos  
            % 
            % Examples:
            % servoAttach(r,11); % attaches servo on pin #11
            % r.servoAttach('S1'); % same as above, attaches servo on pin #11 that is Servo 1 in Ramps
            
            %%%%%%%%%%%%%%%%%%%%%%%%% ARGUMENT CHECKING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % check arguments if r.chkp is true
            if r.chkp
                % check input argument number
                if nargin~=2
                    error('Function must have the "pin" argument');
                end  
                % check pin
                errstr1=rampino.checknum(pin,'servo number',2:11);
                errstr2=rampino.checkstr(pin,'servo number',{'S1','S2','S3','S4'});
                if ~isempty(errstr1)&&~isempty(errstr2)
                    error([errstr1 ' OR ' errstr2])    
                end      
            end 
            %If Alias, convert to number
            rampsPServos=[11,6,5,4];
            if(ischar(pin))
                pin=rampsPServos((find(contains({'s1','s2','s3','s4'},lower(pin)))));
            end
            %%%%%%%%%%%%%%%%%%%%%%%%% ATTACH SERVO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
            % send command, pin and value (1 for attach)
            write(r.aser,[54 97+pin 49],'char');
            % store the servo status
            r.srvs(pin)=1;
            % update pin status to unassigned
            r.pins(pin)=-1; 
        end % servoattach
        
        % servo detach
        function servoDetach(r,pin)  
            % servoDetach(r,pin); detaches a servo from its corresponding pin.
            % The first argument, r, is the rampino object. The second argument, 
            % pin, is the number of the pin where the servo is attached 2 to 11
            % It can also be used the alias 'S1' to 'S4' to match the Ramps servo connectors.
            % See servoAttach for more information on pin limitations.
            %
            % Examples:
            % servoDetach(r,11); % detaches a servo attached on S1, pin 11
            % r.servoDetach('S1'); % detaches a servo attached on S1, pin 11
            
            %%%%%%%%%%%%%%%%%%%%%%%%% ARGUMENT CHECKING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%           
            % check arguments if r.chkp is true
            if r.chkp  
                % check input argument number
                if nargin~=2
                    error('Function must have the "pin" argument');
                end           
                % check servo number
                errstr1=rampino.checknum(pin,'servo number',2:11);
                errstr2=rampino.checkstr(pin,'servo number',{'S1','S2','S3','S4'});
                if ~isempty(errstr1)&&~isempty(errstr2)
                    error([errstr1 ' OR ' errstr2])    
                end              
            end
            %If Alias, convert to number
            rampsPServos=[11,6,5,4];
            if(ischar(pin)) 
                pin=rampsPServos((find(contains({'s1','s2','s3','s4'},lower(pin)))));
            end
            %%%%%%%%%%%%%%%%%%%%%%%%% DETACH SERVO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % send mode, pin and value (0 for detach)
            write(r.aser,[54 97+pin 48+0],'char');
            r.srvs(pin)=0; 
        end % servodetach
        
        % servo read
        function val=servoRead(r,pin)
            % val=servoRead(r,pin); reads the angle of a given servo.
            % The first argument, r, is the rampino object.
            % The second argument, pin, is the number of the pin where the servo is attached 2 to 11
            % It can also be used the alias 'S1' to 'S4' to match the Ramps servo connectors.
            % See servoAttach for more information on pins.
            % The returned value is the angle in degrees, typically from 0 to 180.
            %
            % Examples:
            % val=servoRead(r,10); % reads angle from servo on pin #10
            % val=r.servoRead(9); % reads angle from servo on pin #9
            
            %%%%%%%%%%%%%%%%%%%%%%%%% ARGUMENT CHECKING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % check arguments if r.chkp is true
            if r.chkp
                % check input argument number
                if nargin~=2
                    error('Function must have the servo number argument');
                end  
                % check servo number
                errstr1=rampino.checknum(pin,'servo number',2:11);
                errstr2=rampino.checkstr(pin,'servo number',{'S1','S2','S3','S4'});
                if ~isempty(errstr1)&&~isempty(errstr2)
                    error([errstr1 ' OR ' errstr2])    
                end     
                % get object name
                if isempty(inputname(1))
                    name='object'; 
                else
                    name=inputname(1);
                end      
            end
            %If Alias, convert to number
            rampsPServos=[11,6,5,4];
            rampsTServos={'S1','S2','S3','S4'};
            if(ischar(pin)) 
                pin=rampsPServos((find(contains({'s1','s2','s3','s4'},lower(pin)))));
            end
            if r.chkp && r.srvs(pin)~=1% check status
                if isempty(find(rampsPServos==pin,1)) %if this pin has alias
                    error(['Servo ' num2str(pin) ' is not attached, please use ' name' '.servoAttach(' num2str(pin) ') to attach it']);
                else
                    error(['Servo ' num2str(pin) ' (' rampsTServos{find(rampsPServos==pin,1)} ') is not attached, please use '...
                        name' '.servoAttach(' num2str(pin) ') OR '  name' '.servoAttach(' rampsTServos{find(rampsPServos==pin,1)} ') to attach it']);
                end    
            end
            %%%%%%%%%%%%%%%%%%%%%%%%% READ SERVO ANGLE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            
            % send command and pin
            write(r.aser,[55 97+pin],'char');
            % get value
            val=read(r.aser,1,'uint8');
        end % servoread
        
        % servo write
        function servoWrite(r,pin,val)       
            % servoWrite(r,pin,val); writes an angle on a given servo.
            % The first argument, r, is the rampino object.
            % The second argument, pin, is the number of the pin where the servo is attached 2 to 11
            % It can also be used the alias 'S1' to 'S4' to match the Ramps servo connectors.
            % See servoAttach for more information on pins.
            % The third argument is the angle in degrees, typically from 0 to 180.
            % 
            % Examples:
            % servoWrite(r,10,45); % rotates servo on pin #10 to 45 degrees
            % r.servoWrite(10,70); % rotates servo on pin #9 to 70 degrees
            
            %%%%%%%%%%%%%%%%%%%%%%%%% ARGUMENT CHECKING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
            % check arguments if r.chkp is true
            if r.chkp
                % check input argument number
                if nargin~=3
                    error('Function must have the servo number and angle arguments');
                end 
                % check servo number
                errstr1=rampino.checknum(pin,'servo number',2:11);
                errstr2=rampino.checkstr(pin,'servo number',{'S1','S2','S3','S4'});
                if ~isempty(errstr1)&&~isempty(errstr2)
                    error([errstr1 ' OR ' errstr2])    
                end   
                % check angle value
                errstr=rampino.checknum(val,'angle',0:180);
                if ~isempty(errstr), error(errstr); end        
                % get object name
                if isempty(inputname(1))
                    name='object';
                else
                    name=inputname(1);
                end               
            end
            %If Alias, convert to number
            rampsPServos=[11,6,5,4];
            rampsTServos={'S1','S2','S3','S4'};
            if(ischar(pin)) 
                pin=rampsPServos((find(contains({'s1','s2','s3','s4'},lower(pin)))));
            end
            if r.chkp && r.srvs(pin)~=1% check status
                if isempty(find(rampsPServos==pin,1)) %if this pin has no alias
                    error(['Servo ' num2str(pin) ' is not attached, please use ' name' '.servoAttach(' num2str(pin) ') to attach it']);
                else %has alias
                    error(['Servo ' num2str(pin) ' (' rampsTServos{find(rampsPServos==pin,1)} ') is not attached, please use '...
                        name' '.servoAttach(' num2str(pin) ') OR '  name' '.servoAttach(' rampsTServos{find(rampsPServos==pin,1)} ') to attach it']);
                end    
            end
            %%%%%%%%%%%%%%%%%%%%%%%%% WRITE ANGLE TO SERVO %%%%%%%%%%%%%%%%%%%%%%%%%%%%                    
            % send command, pin and value
            write(r.aser,[56 97+pin val],'char');           
        end % servowrite
        
        % encoder attach
        function encoderAttach(r,enc,pinA,pinB)           
            % encoderAttach(r,enc,pinA,pinB); attaches an encoder to 2 pins.
            % The first argument, r, is the rampino object.
            % The second argument is the encoder's number, either 1,2 or 3.
            % The third and fourth arguments, pinA and pinB, are the number of 
            % the pins where the encoder's pinA and pinB are attached 
            % These pins need to support interrups that in the Mega are 2,3,18,19,20,21 
            %
            % Examples:
            % encoderAttach(r,1,2,3); % attaches encoder #1 on pins 2 and 3
            % r.encoderAttach(2,18,19); % attaches encoder #2 on pins 18 and 19
            
            %%%%%%%%%%%%%%%%%%%%%%%%% ARGUMENT CHECKING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % check arguments if r.chkp is true
            if r.chkp
                % check nargin
                if nargin~=4
                    error('Function must have the encoder number, pin A and pin B arguments');
                end
                % check encoder number
                errstr=rampino.checknum(enc,'encoder number',1:3);
                if ~isempty(errstr), error(errstr); end
                % check pin A
                errstr=rampino.checknum(pinA,'pin A',[2 3 18 19 21 20]);
                if ~isempty(errstr), error(errstr); end
                % check pin B
                errstr=rampino.checknum(pinB,'pin B',[2 3 18 19 21 20]);
                if ~isempty(errstr), error(errstr); end
            end
            % silently detach servos on the correspinding pins
            tmp=r.chkp;
            r.chkp=0;
            r.servoDetach(pinA);
            r.servoDetach(pinB);
            r.chkp=tmp;
            %%%%%%%%%%%%%%%%%%%%%%%%% ATTACH ENCODER %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
            % send mode, encoder and pins
            write(r.aser,[69 48+enc 97+pinA 97+pinB],'char');
            % store the encoder status
            r.encs(enc)=1;
            % update pin status to input
            r.pins([pinA pinB])=[0 0];
        end % encoderattach
        
        % encoder detach
        function encoderDetach(r,enc)         
            % encoderDetach(r,enc); detaches an encoder from its pins.
            % The first argument before the function name, a, is the rampino object.
            % The second argument, enc, is the number of the encoder to
            % be detached (1, 2, or 3). This also detaches any interrupt
            % routine previously attached to the specified pins.
            %
            % Examples:
            % encoderDetach(r,1); % detach encoder #1
            % r.encoderDetach(2); % detach encoder #2
            
            %%%%%%%%%%%%%%%%%%%%%%%%% ARGUMENT CHECKING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
            % check arguments if r.chkp is true
            if r.chkp
                % check nargin
                if nargin~=2
                    error('Function must have the encoder number argument');
                end
                % check encoder number
                errstr=rampino.checknum(enc,'encoder number',1:3);
                if ~isempty(errstr), error(errstr); end
            end            
            %%%%%%%%%%%%%%%%%%%%%%%%% DETACH ENCODER %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % send mode, encoder and value
            write(r.aser,[70 48+enc],'char');
            r.encs(enc)=0;          
        end % encoderdetach
        
        % encoder status
        function val=encoderStatus(r,enc)
            % encoderStatus(r,enc); Reads the status of an encoder if (attached/detached).
            % The first argument, r, is the rampino object. The second argument, enc, 
            % is the number of the encoder (1 to 3) which status needs to be read.
            % The returned value is either 1 (encoder attached) or 0 (encoder detached).
            % Called without output arguments, the function prints a string specifying
            % the status of the encoder. Called without input arguments, the function
            % either returns the status vector or prints the status of each encoder.
            %
            % Examples:
            % val=encoderStatus(r,1); % return the status of encoder #1
            % encoderStatus(r,2); % prints the status of encoder #2
            % encoderStatus(r); % prints the status of all encoders
            % r.encoderStatus; % prints the status of all encoders
            
            %%%%%%%%%%%%%%%%%%%%%%%%% ARGUMENT CHECKING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % check nargin if r.chkp is true
            if r.chkp
                if nargin>2
                    error('Function cannot have more than one argument (encoder number) beyond the object name');
                end
            end
            % with no arguments calls itself recursively for all encoders
            if nargin==1
                if nargout>0
                    val=zeros(3,1);
                    for i=1:3
                        val(i+1)=r.encoderStatus(i);
                    end
                    return
                else
                    for i=1:3
                        r.encoderStatus(i);
                    end
                    return
                end
            end
            % check encoder number if r.chkp is true
            if r.chkp
                errstr=rampino.checknum(enc,'encoder number',1:3);
                if ~isempty(errstr), error(errstr); end
            end
            % gets value from the encoder state vector
            val=r.encs(enc);
            if nargout==0
                str={'DETACHED','ATTACHED'};
                disp(['Encoder ' num2str(enc) ' is ' str{1+val}]);
                clear val
                return
            end     
        end % encoderstatus
        
        % encoder read
        function val=encoderRead(r,enc) 
            % val=encoderRead(r,enc); reads the position of a given encoder.
            % The first argument, r, is the rampino object.
            % The second argument, enc, is the encoder number (1 to 3).
            % The returned value is the position in steps, where clockwise
            % rotation are assumed positive by convention.
            %
            % Examples:
            % val=encoderRead(r,2); % reads current step from encoder #2
            % val=r.encoderRead(1); % reads current step from encoder #1
            
            %%%%%%%%%%%%%%%%%%%%%%%%% ARGUMENT CHECKING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % check arguments if r.chkp is true
            if r.chkp 
                % check nargin
                if nargin~=2
                    error('Function must have the encoder number argument');
                end
                % check encoder number
                errstr=rampino.checknum(enc,'encoder number',1:3);
                if ~isempty(errstr), error(errstr); end
                % get object name
                if isempty(inputname(1))
                    name='object';
                else
                    name=inputname(1);
                end
                % check status
                if r.encs(enc)~=1
                    error(['Encoder ' num2str(enc) ' is not attached, please use ' name' '.encoderAttach(' num2str(enc) ') to attach it']);
                end
            end
            
            %%%%%%%%%%%%%%%%%%%%%%%%% READ ENCODER POSITION %%%%%%%%%%%%%%%%%%%%%%%%%%%              
            % send mode and enc
            write(r.aser,[71 48+enc],'char');
            % get value
            val=read(r.aser,1,'int16');
        end % encoderread
        
        % encoder reset
        function encoderReset(r,enc)     
            % encoderReset(r,enc); resets the position of a given encoder to 0 (Home Position).
            % The first argument, r, is the rampino object.
            % The second argument, enc, is the number of the encoder (1 to 3).
            %
            % Examples:
            % encoderReset(r,1); % resets encoder #1
            % r.encoderReset(2); % resets encoder #2
            
            %%%%%%%%%%%%%%%%%%%%%%%%% ARGUMENT CHECKING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % check arguments if r.chkp is true
            if r.chkp
                % check nargin
                if nargin~=2
                    error('Function must have the encoder number argument');
                end
                % check encoder number
                errstr=rampino.checknum(enc,'encoder number',1:3);
                if ~isempty(errstr), error(errstr); end
                % get object name
                if isempty(inputname(1))
                    name='object';
                else
                    name=inputname(1);
                end
                % check status
                if r.encs(enc)~=1
                    error(['Encoder ' num2str(enc) ' is not attached, please use ' name' '.encoderAttach(' num2str(enc) ') to attach it']);
                end
            end 
            %%%%%%%%%%%%%%%%%%%%%%%%% RESET ENCODER POSITION %%%%%%%%%%%%%%%%%%%%%%%%%%         
            % send mode and encoder number
            write(r.aser,[72 48+enc],'char');            
        end % encoderreset
        
        % encoder attach
        function encoderDebounce(r,enc,del)
            % encoderDebounce(r,enc,del); sets debounce delay for an encoder.
            % The first argument, a, is the rampino object.
            % The second argument is the encoder's number, either 1,2 or 3.
            % The third argument, del, is the debounce delay (in units of
            % approximately 0.1 ms each) between the instant in which an 
            % interrupt is triggered on either pinA or pinB and the instant
            % in which the status (HIGH or LOW) of the corresponding pin is
            % actually read to find the rotation direction.
            % 
            % Note that this delay will limit the maximum rotation rate that
            % can be detected, and is intended for simple manual mechanical encoders.
            % Motor optical modern encoders don't need to set this extra delay. 
            % Possible values go from 0 (no delay) to 69 (6.9 ms). In general, 
            % it is suggested to keep this value no higher than 20 (2 ms). 
            %
            % Examples:
            % encoderDebounce(r,3,20); % sets a delay of 2mS for encoder #3
            % r.encoderDebounce(1,17); % sets a delay of 1.7mS for encoder #1
            
            
            %%%%%%%%%%%%%%%%%%%%%%%%% ARGUMENT CHECKING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % check arguments if r.chkp is true
            if r.chkp        
                % check nargin
                if nargin~=3
                    error('Function must have both the encoder number and the delay value as arguments');
                end
                % check encoder number
                errstr=rampino.checknum(enc,'encoder number',1:3);
                if ~isempty(errstr), error(errstr); end
                % check delay value
                errstr=rampino.checknum(del,'del',0:69);
                if ~isempty(errstr), error(errstr); end
                % get object name
                if isempty(inputname(1))
                    name='object'; 
                else
                    name=inputname(1);
                end
                % check status
                if r.encs(enc)~=1
                    error(['Encoder ' num2str(enc) ' is not attached, please use ' name' '.encoderAttach(' num2str(enc) ') to attach it']);
                end            
            end
            %%%%%%%%%%%%%%%%%%%%%%%%% SETS DEBOUNCE DELAY %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % send mode, encoder and pins
            write(r.aser,[73 48+enc 97+del],'char');                        
        end % encoderdebounce
        
        % Tone Play
        function tonePlay(r,pin,frequency,duration)       
            % tonePlay(r,pin,frequency,duration)  Play tone in a given pin
            % The first argument, r, is the rampino object.
            % The second argument, pin, is the number of the pin where the speaker is connected
            % Only 3 pins can be used simultaneously as the Arduino hadware dedicates a hardware timer to each
            % The third argument is the frequency in hertzs, from 0 to 65535. A 0 means stop the tone
            % The fourth argument is the duration in mS, from 0 to 65535. A 0 means play continously until frequency 0
            % or a call without frequency or duration
            % 
            % Examples:
            % tonePlay(r,10,1000,2000); % Play a tone of 1000Hz at pin 10 for 2 seconds
            % r.tonePlay(23,100,1000); % Play a tone of 100Hz at pin 23 for 1 second
            % r.tonePlay(23,100,0); % Play a tone of 100Hz at pin 23 until stop...
            % r.tonePlay(23,100);   % same, play a tone of 100Hz at pin 23 until stop
            % r.tonePlay(23,0); % Stop the tone at pin 23...
            % r.tonePlay(23);   % same, stop the tone at pin 23
            
            %%%%%%%%%%%%%%%%%%%%%%%%% ARGUMENT CHECKING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
            %default
            if nargin==2
                frequency=0;
                duration=0;
            elseif nargin==3
                duration=0;
            end
            % check arguments if r.chkp is true
            if r.chkp
                % check input argument number
                if nargin<2
                    error('Function must have the pin argument at least');
                end 
                % check pin number
                errstr=rampino.checknum(pin,'pin number',2:69);
                if ~isempty(errstr), error(errstr); end
                % check frequency value
                if nargin>2 
                    if (frequency < 0 || frequency > 65535)
                        error('Value of frequency is out of range, must be from 0 to 65535' );
                    elseif (duration < 0|| duration > 65535)
                        error('Value of duration is out of range, must be from 0 to 65535' );
                    end
                end 
            end         
            %%%%%%%%%%%%%%%%%%%%%%%%% PLAY TONE TO PIN %%%%%%%%%%%%%%%%%%%%%%%%%%%%                    
            % send command, pin and frequency and duration parameters
            write(r.aser,[58 97+pin],'char');
            write(r.aser,[frequency duration],'uint16');
        end % playTone
 
  %%   DC Motors (To be tested)
        % DC motor attach
        function DCMotorAttach(r,motor,flip)           
            % DCMotorAttach(r,motor,flip); attaches and enables a motor to its Ramps board 
            % fixed pins, and enable. The first argument, r, is the rampino object. 
            % The second argument is the number of the DC motor,which can be
            % from 1 to 5 (these coresponds to X,Y,Z,E0,E1 respectivelly on the Ramps shield)
            % A DC motor driver such as MAX14870 must be connected to these module headers
            % The third argument is optional. If used, sets direction flipped for this motor (value 1)
            % 
            % Examples:
            % DCMotorAttach(r,1); % attaches and enables DC motor X in Ramps 
            % r.DCMotorAttach(5);  % attaches and enables DC motor E0 in Ramps
            % r.DCMotorAttach(5,1); % attaches and enables DC motor E0 in Ramps,
            %                         then the velocity direction is flipped in this motor
            
            %%%%%%%%%%%%%%%%%%%%%%%%% ARGUMENT CHECKING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % check arguments if r.chkp is true
            if r.chkp
                % check nargin
                if nargin <2
                    error('Function must have the motor number');
                end
                % check motor number
                errstr=rampino.checknum(motor,'motor number',1:5);
                if ~isempty(errstr), error(errstr); end
                % check for mode
                if nargin==3
                    errstr=rampino.checknum(flip,'Flip Mode',0:1);
                    if ~isempty(errstr), error(errstr); end
                end
            end
            if nargin==2
                flip=0; %Default
            end
            %%%%%%%%%%%%%%%%%%%%%%%% ATTACH DC MOTOR %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
            % send command, motor and pins
            write(r.aser,[59 48+motor 49+flip],'char');
            % store the motor status
            r.dcs(motor)=flip+1; % Fill the DC motor status with flip mode
        end % DCMotorAttach 
        
        % DC motor detach
        function DCMotorDetach(r,motor)           
            % DCMotorDetach(r,motor); detaches and disable a motor on its Ramps board 
            % fixed pins, and enable. The first argument, r, is the rampino object. 
            % The second argument is the number of the DC motor,which can be
            % from 1 to 5 (these coresponds to X,Y,Z,E0,E1 respectivelly on the Ramps shield)
            % A DC motor driver such as MAX14870 must be connected to these module headers
            % 
            % Examples:
            % DCMotorDetach(r,1); % Detaches and disables DC motor X in Ramps 
            % r.DCMotorDetach(5);  % Detaches and disables DC motor E0 in Ramps
            
            %%%%%%%%%%%%%%%%%%%%%%%%% ARGUMENT CHECKING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % check arguments if r.chkp is true
            if r.chkp
                % check nargin
                if nargin <2
                    error('Function must have the motor number');
                end
                % check motor number
                errstr=rampino.checknum(motor,'motor number',1:5);
                if ~isempty(errstr), error(errstr); end
                % check for mode
            end
            %%%%%%%%%%%%%%%%%%%%%%%% DETACH DC MOTOR %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
            % send command, motor and pins
            write(r.aser,[59 48+motor 48],'char');
            % store the motor status
            r.dcs(motor)=0; % Fill the DC motor status with flip mode
        end % DCMotorDetach 

        %DCMotor Status
        function val=DCMotorStatus(r,motor)     
            % DCMotorStatus(r,pin); Reads if a DC motor is attached, either 
            % attached, with flipped direction, or detached.
            % The first argument, r, is the rampino object. 
            % The second argument is the numberof the stepper motor,which can be
            % from 1 to 5 (these coresponds to X,Y,Z,E0,E1 respectivelly on the Ramps shield)
            % Numeric status returned when output argument is used:
            %   0 if the motor is detached, 1 if attached, 2 if it is attached and flipped 
            % If no output argument, A string is displayed saying the status
            %
            % Examples:
            % val=DCMotorStatus(r,5); % return the status of DC Motor 5: 0, 1, or 2
            % DCMotorStatus(r,5); % prints a string with the status of DC Motor 5
            % DCMotorStatus(r); % prints the status of all DC Motor motors
            % r.DCMotorStatus; % prints the status of all DC Motor motors
            
            %%%%%%%%%%%%%%%%%%%%%%%%% ARGUMENT CHECKING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%      
            % check nargin if r.chkp is true 
            if r.chkp         
                % check input argument number
                if nargin>2
                    error('Function cannot have more than one argument (motor number) beyond the object name');
                end
            end
            % with no arguments calls itself recursively for all motors
            if nargin==1
                if nargout>0
                    val=zeros(5);
                    for i=1:5
                        val(i)=r.DCMotorStatus(i);
                    end
                    return
                else
                    for i=1:5
                        r.DCMotorStatus(i);
                    end
                    return
                end
            end
            % check motor number if r.chkp is true
            if r.chkp
                errstr=rampino.checknum(motor,'motor number',1:5);
                if ~isempty(errstr), error(errstr); end
            end 
            %%%%%%%%%%%%%%%%%%%%%%%%% ASK DC MOTOR STATUS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % send command and pin
            write(r.aser,[60 48+motor],'char');
            % check answer
            val=read(r.aser,1,'char');  %Return status     
            % updates the DC Motor status vector 2nd field (absolute steps remaining, or -1 if dettached)
            r.dcs(motor)=val;
            if nargout==0 %Without output argument
                if val==0
                    disp(['DC Motor ' num2str(motor) ' is DETACHED.']);
                else
                    if r.dcs(motor)==2
                        disp(['DC Motor ' num2str(motor) ' is ATTACHED and flipped mode']);
                    elseif r.dcs(motor)==1
                        disp(['DC Motor ' num2str(motor) ' is ATTACHED']);
                    else
                        error('Value mismatch between hardware mode locally and rampino')
                    end
                end
                clear val
                return
            else   %With output argument
                val = r.dcs(motor); %return if dettached=0, attached=1, or flip attached=2
            end       
        end % DCMotorStatus              
        
        % DC Motor velocity
        function val=DCMotorVelocity(r,motor,val)         
            % val=DCMotorVelocity(r,motor,val); sets the velocity and direction of a given DC motor
            % The first argument, r, is the rampino object.
            % The second argument, motor, is the number of the stepper motor,
            % which can be from 1 to 5 (these coresponds to X,Y,Z,E0,E1 respectivelly
            % on the Ramps shield). Assume pwm/dir type controllers as MAX14870.
            % The third argument is the vector velocity from -100% to 100% (pwm duty).
            % 0 stop the motor in brake mode, to leave it floating, a DCMotorDetach has to be called 
            % Called as DCMotorVelocity(r,motor), it returns the numeric velocity at which 
            % the given motor is set to run. If there is no output
            % argument, it prints the velocity of the DC motor in text.
            % Called as DCMotorVelocity(r), it prints the velocity of all DC motors.
            %
            % Examples:
            % DCMotorVelocity(r,2,50);      % sets velocity of DC motor 2 as 50% PWM duty
            % val=DCMotorVelocity(r,1);    % returns the numeric velocity of DC motor 1
            % DCMotorVelocity(r,2);        % prints the velocity of DC motor 2
            % DCMotorVelocity(r);          % prints the velocity of all DC motor
            % r.DCMotorVelocity;           % prints the velocity of all DC motor
            
            %%%%%%%%%%%%%%%%%%%%%%%%% ARGUMENT CHECKING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % check arguments if r.chkp is true
            if r.chkp
                % check nargin
                if nargin>3
                    error('This function cannot have more than 3 arguments, object, DC motor number and velocity');
                end
                % if DC motor number is there check it
                if nargin>1
                    errstr=rampino.checknum(motor,'DC motor number',1:5);
                    if ~isempty(errstr), error(errstr); end
                end
                % if velocity argument is there check it
                if nargin>2
                    errstr=rampino.checknum(val,'velocity',-100:100);
                    if ~isempty(errstr), error(errstr); end
                end   
            end
            % perform the requested action
            if nargin==3 
                %%%%%%%%%%%%%%%%%%%%%%%%% SET DC Motor Velocity %%%%%%%%%%%%%%%%%%%%%%%%%%%                
                % send command, motor and value
                write(r.aser,[61 48+motor val],'int8'); 
                % store locally Velocity value in case it needs to be reported
                r.dcv(motor)=val;
                % clear val if is not needed as output
                if nargout==0
                    clear val;
                end               
            elseif nargin==2
                if nargout==0
                    % print velocity value
                    disp(['The velocity of DC Motor number ' num2str(motor) ' is set to: ' num2str(r.dcv(motor)) ' % PWM Duty Cycle']);
                else
                    % return velocity value
                    val=r.dcv(motor);
                end
            else
                if nargout==0
                    % print velocity value for each DC Motor
                    for motor=1:5
                        disp(['The velocity of DC Motor number ' num2str(motor) ' is set to: ' num2str(r.sspd(motor)) ' % PWM Duty Cycle']);
                    end
                else
                    % return velocity values
                    val=r.dcv;
                end
            end
        end % DCMotorVelocity
        
%%    Stepper Motors 
        % Stepper motor attach
        function stepperAttach(r,motor,hardwareMode)           
            % stepperAttach(r,motor,hardwareMode); attaches a motor to its Ramps board 
            % fixed pins, and enable. The first argument, r, is the rampino object. 
            % The second argument is the numberof the stepper motor,which can be
            % from 1 to 5 (these coresponds to X,Y,Z,E0,E1 respectivelly on the Ramps shield)
            % The third argument is optional. If used, tells if hardware mode is on (value 1)
            % or if software mode is on, value 0 or omitted. In software he signals in the SPI pins corresponding 
            % to MS1 and MS2, plus another pin connected to MS3 are set as outputs to control the mode in each motor.
            % SPI signals are shared in all motors, but the MS3 is individual. Jumpers must be placed across
            % these three signals on the black 2X4 header under the motor driver modules to operate in software mode, read notes. 
            % Software mode provides more flexibility, specially if we expect swithing modes in motors via software,
            % but it requieres to give up the SPI bus, plus an output pin for each motor
            %**NOTE1: In Software mode, the MODE or MS Jumpers under the drivers must be placed across the black headers
            %  Following Ramps 1.6 Plus datasheet jumper drawing, MS3, MS2, and MS1 go across pins 3-5, 2-12, and 1-11 respectivelly. 
            %**NOTE2: In Hardware mode similarly some jumpers under the driver modules may need to placed between white headers and
            %  one side of black header, following the Micro Step combination shown in the modules documentaion.
            %  The jumpers MS3, MS2, and MS1 must follow the binary form of Microset Resolution desired.
            %  Example: Eighth Step is value 3, binary 011, or Low, High, High in table, so only MS2 and MS1 jumpers
            %  are placed across pins 9-12, and 10-11 respectivelly. In hardware mode still is needed to set the corresponding
            %  microstepping with the stepperStep() command mode parameter or wrong speed will be calculated.
            %**NOTE3: The fourth jumper between 4-6 that controls nRST and nSLEEP is not used by Rampino at this point
            % 
            % Examples:
            % r.stepperAttach(1,1); % attaches and enables stepper motor X in Ramps, hardware mode 
            % r.stepperAttach(5); % attaches and enables stepper motor E0 in Ramps, software mode
            
            %%%%%%%%%%%%%%%%%%%%%%%%% ARGUMENT CHECKING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % check arguments if r.chkp is true
            if r.chkp
                % check nargin
                if nargin <2
                    error('Function must have the motor number');
                end
                % check motor number
                errstr=rampino.checknum(motor,'motor number',1:5);
                if ~isempty(errstr), error(errstr); end
                % check for mode
                if nargin==3
                    errstr=rampino.checknum(hardwareMode,'Hardware Mode',0:1);
                    if ~isempty(errstr), error(errstr); end
                end
            end
            if nargin==2
                hardwareMode=0; %Default
            end
            %%%%%%%%%%%%%%%%%%%%%%%% ATTACH STEPPER MOTOR %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
            % send command, motor and pins
            write(r.aser,[65 48+motor 49+hardwareMode],'char');
            % store the motor status
         %   r.steps(motor,1)=hardwareMode+1; % Fill the stepper status with harware mode (1st field 1 is software, 2 hardware))
        end % stepperAttach 
        
        % Stepper motor detach
        function stepperDetach(r,motor)           
            % stepperDetach(r,motor,number); detaches a motor from its Ramps board 
            % fixed pins, and disable. So other fuction could use the same pins
            % The first argument, r, is the rampino object. 
            % The second argument is the numberof the stepper motor,which can be
            % from 1 to 5 (these coresponds to X,Y,Z,E0,E1 respectivelly on the Ramps shield)
            %
            % Examples:
            % stepperDetach(r,1); % detaches and disables stepper motor X in Ramps 
            % r.stepperDetach(5); % detaches and disables stepper motor E0 in Ramps
            
            %%%%%%%%%%%%%%%%%%%%%%%%% ARGUMENT CHECKING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % check arguments if r.chkp is true
            if r.chkp
                % check nargin
                if nargin~=2
                    error('Function must have the motor number');
                end
                % check motor number
                errstr=rampino.checknum(motor,'motor number',1:5);
                if ~isempty(errstr), error(errstr); end
            end
            %%%%%%%%%%%%%%%%%%%%%%%% DETACH STEPPER MOTOR %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
            % send command, motor and pins
            write(r.aser,[65 48+motor 48],'char');
            % store the motor status
        %    r.steps(motor,1)=0; % Fill the stepper status with harware mode (1st field 0 is detached)
        end % stepperDetach 
        
        %Stepper Status
        function [rSteps,attach]=stepperStatus(r,motor)     
            % StepperStatus(r,pin); Reads if a stepper is detached or the number.
            % of steps remaining to complete a run. 
            % The first argument, r, is the rampino object. 
            % The second argument is the numberof the stepper motor,which can be
            % from 1 to 5 (these coresponds to X,Y,Z,E0,E1 respectivelly on the Ramps shield)
            % One or Two parameters are returned when output arguments are used:
            %   With a single parameter, returns -1 if the motor is reported detached, or absolute number of steps remaining to go if attached  
            %   If a second parameter is assigned, returns 0 if the motor is detached, 1 if attached and in software mode, 2 if it is set by hardware jumpers, 
            % If no output argument is used, a text string is printed saying the mode, and absolute steps remaining if attached.  
            %
            % Examples:
            % [steps,attached]=r.stepperStatus(5); % returns the steps remaining and attachment status for motor 5
            % steps=r.stepperStatus(3); % returns the steps remaining for motor 5
            % stepperStatus(r,5); % prints the status of stepper 5
            % stepperStatus(r); % Text prints the status of all stepper motors
            % r.stepperStatus; % as above, text prints the status of all stepper motors
            
            %%%%%%%%%%%%%%%%%%%%%%%%% ARGUMENT CHECKING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%      
            % check nargin if r.chkp is true 
            if r.chkp         
                % check input argument number
                if nargin>2
                    error('Function cannot have more than one argument (motor number) beyond the object name');
                end
            end
            % with no arguments calls itself recursively for all motors
            if nargin==1
                if nargout==1
                    rSteps=zeros(1,5);
                    for i=1:5
                        %val(i,:)=r.stepperStatus(i);
                        rSteps(i)=r.stepperStatus(i);
                    end
                    return
                elseif nargout==2
                    rSteps=zeros(5,1);
                    attach=zeros(5,1);
                    for i=1:5
                        [rSteps(i),attach(i)] = r.stepperStatus(i);
                    end
                    return
                else % No output argument case
                    for i=1:5
                        r.stepperStatus(i);
                    end
                    return
                end
            end
            % check motor number if r.chkp is true
            if r.chkp
                errstr=rampino.checknum(motor,'motor number',1:5);
                if ~isempty(errstr), error(errstr); end
            end 
            %%%%%%%%%%%%%%%%%%%%%%%%% ASK STEPPER MOTOR STATUS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            write(r.aser,[66 48+motor],'char'); % asking steps status and motor
            rSteps=read(r.aser,1,'int32');  %Red steps return as int32 (4 bytes) 

            % updates the stepper status vector 2nd field (absolute steps remaining, or -1 if dettached)
            %--r.steps(motor,2)=val;
            
            if nargout==0 %Without output argument    
                if rSteps==-1
                    disp(['Stepper Motor ' num2str(motor) ' is DETACHED.']);
                else
                    write(r.aser,[66 68+motor],'char'); % asking attach  status and motor
                    attach=read(r.aser,1,'uint8')-48;  %Red attachment status return as char  
                    %if r.steps(motor,1)==2
                    if attach==2
                        disp(['Stepper Motor ' num2str(motor) ' is in hardware mode and has ' num2str(rSteps) ' steps left.' ]);
                    %elseif r.steps(motor,1)==1
                    elseif attach==1
                        disp(['Stepper Motor ' num2str(motor) ' is in software mode and has ' num2str(rSteps) ' steps left.' ]);
                    else
                        error('Value mismatch between hardware mode locally and rampino')
                    end
                end
                clear rSteps;
                %clear val
                %return
            elseif nargout==1   
               % return
                %rSteps = r.steps(motor,2); %return Steps remaining (-1) if detacched 
            elseif nargout==2  
                %rSteps = r.steps(motor,2);
                %attach = r.steps(motor,1);%return 2 parameters per motor. Attach Status, then steps remaining with (-1) if detacched 
                write(r.aser,[66 68+motor],'char'); % asking attach  status and motor
                attach=read(r.aser,1,'uint8')-48;  %Red attachment status return as char  
            else
                error('Too many output arguments')
            end       
        end % stepperStatus       
        
        % stepper speed
        function val=stepperSpeed(r,motor,val,linear,acc,dec)         
            % val=stepperSpeed(r,motor,val,linear,acc,dec); sets the speed of a given stepper motor
            % The first argument, r, is the rampino object.
            % The second argument, motor, is the number of the stepper motor,
            % which can be from 1 to 5 (these coresponds to X,Y,Z,E0,E1 respectivelly
            % on the Ramps shield). Assume step/dir type controllers and 200 full steps motors.
            % The third argument (op) is the RPM speed from 0 (stop with brake) to 255 (maximum).
            % The forth argument (op) is if the motion is constant (default) 0 or linear 1
            % The fifth and sith (op) is used along with linear motion to define acceleration and deceleration in steps/s2 (1 to 16383 max)
            % If acceleration and deceleration are not defined with linear motion, last value is used.
            % Called as stepperSpeed(r,motor), it returns the numeric speed at which 
            % the given motor is set to run, and optional parametrs. If there is no output
            % argument it prints the speed of the stepper motor.
            % Called as stepperSpeed(r), it prints the speed of all stepper motors.
            %
            % Note: You must use the stepperStep function to actually run
            % the motor at the given speed, either forward or backwards (or release it). 
            %
            % Examples:
            % stepperSpeed(r,2,50,1,500,1000)   % sets speed of stepper 2 as 50 rpm, Linear mode, acceleartion at 500 and deceleration 1000 steps s^2
            % stepperSpeed(r,1,60,0);           % sets speed of stepper 1 at 60 rpm, constant mode  
            % stepperSpeed(r,2);                % prints the parameters of stepper 2 in text form
            % stepperSpeed(r);                  % prints the parameters of all steppers in text form
            % x=r.stepperSpeed;                 % returns the numerical parameters of all steppers
            
            %%%%%%%%%%%%%%%%%%%%%%%%% ARGUMENT CHECKING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % check arguments if r.chkp is true
            if r.chkp
                % check nargin
                if nargin>6
                    error('This function cannot have more than 6 arguments, object, stepper number, speed, linear mode, acceleration, deceleration ');
                end
                % if stepper number is there check it
                if nargin>1
                    errstr=rampino.checknum(motor,'stepper number',1:5);
                    if ~isempty(errstr), error(errstr); end
                end
                % if speed argument is there check it
                if nargin>2
                    errstr=rampino.checknum(val,'speed',0:255);
                    if ~isempty(errstr), error(errstr); end
                end   
                % if linear mode argument is there check it
                if nargin>3
                    errstr=rampino.checknum(linear,'Linear motion',0:1);
                    if ~isempty(errstr), error(errstr); end
                end   
                % if linear motion accel, decel are there  check them
                if nargin>4 && (acc < 1 || acc > 16383)     
                    error('Value of acceleration out of range, must be from 1 to 16383' );
                end
                if nargin>5 && (dec < 1 || dec > 16383)   
                    error('Value of deceleration out of range, must be from 1 to 16383' );
                end
            end
            %defaults and action
            prof={'CONSTANT' 'LINEAR'};
            switch nargin
                case 1 %only object, return parameters of all motors 
                    if nargout==0
                        % print speed value for each stepper 5X4
                        for motor=1:5
                            r.stepperSpeed(motor);
                        end
                    else
                        % return all status speed parameters 5x5 
                        val=r.sspd; 
                    end
                    return;
                case 2 %includes motor number, so return parameters for this motor number                
                    if nargout==0
                        % print speed paramers "Stepper 1 parameters: 60 RPM, LINEAR, , Accel 1000 Steps s^2, Decel 1000 Steps s^2"
                        disp(['Stepper ' num2str(motor) ' parameters: ' num2str(r.sspd(motor,1)) ' RPM, '  prof{r.sspd(motor,2)+1}...
                              ', Accel '  num2str(r.sspd(motor,3)) ' Steps s^2, Decel ' num2str(r.sspd(motor,4)) ' Steps s^2' ]);
                    else
                        % return parameters, for this motor 1X4
                        val=r.sspd(motor,:);
                    end
                    return
                % 3 to 6 store locally parameters to report transmit
                case 3 %Includes also motor speed, so apply it with all other curent last parameters (or default if first time)
                    r.sspd(motor,1)=val;
                case 4 %Includes also motion mode, LINEAR or CONSTANT. This last ignores further parameters
                    r.sspd(motor,1)=val; r.sspd(motor,2)=linear; 
                case 5 %Includes acceleration
                    r.sspd(motor,1)=val; r.sspd(motor,2)=linear; r.sspd(motor,3)=acc; 
                case 6 %includes deceleration
                    r.sspd(motor,1)=val; r.sspd(motor,2)=linear; r.sspd(motor,3)=acc; r.sspd(motor,4)=dec; 
            end  
            % perform the requested action
            %%%%%%%%%%%%%%%%%%%%%%%%% SET STEPPER SPEED %%%%%%%%%%%%%%%%%%%%%%%%%%%                
            % send command, motor and value
            write(r.aser,[67 48+motor r.sspd(motor,1) r.sspd(motor,2)],'char');
            write(r.aser,[r.sspd(motor,3) r.sspd(motor,4)],'uint16');
            
            % clear val if is not needed as output
            if nargout==0
                clear val;
            end               
        end % stepperspeed
        
        % stepper step
        function stepperStep(r,motor,op,steps,mode,pin,pol)        
            % stepperStep(r,motor,op,steps,mode,pin); runs a given stepper motor
            % The first argument, r, is the rampino object. The second argument, 
            % motor, is the number of the stepper motor, which can be from 1 to 5
            % (these coresponds to X,Y,Z,E0,E1 respectivelly on the Ramps shield).
            % Assume step/dir type controllers and 200 full steps motors.
            % Popular compatible controller modules for the Ramps are the Pololu A4988 and DRV8825.
            % The third argument, op for operation can be:
            %   0, Disable driver, just releases the shaft driving (this can advance extra steps if there is enough momentum)
            %   1, Stop motion inmediatly and while enable
            %   2, Start braking (deceleration and stop) early in Linear mode. For constant speed, this is the same as stop()
            %   8, Enable the controler driver and go with the steps and microstep mode 
            %   9, Keep Enable the controler and stepper moving, but add addtional steps steps
            %   10,Enable the controler driver and go with the steps and microstep mode, then autodisable. Op 10 and 11 will 
            %      wait until steps are completed to do an enabled (braked) stop, then after a brief delay autodisable
            %   11,Keep Enable the controler and stepper moving, but add addtional steps steps, then autodisable 
            % WARNING: When using ops 8 or 9, remember to disable soon after the operation is completed, or overheating
            % may occur in the motor if the current is not limited properly. Ops 10 and 11 are more secure for this purpose   
            % NOTE: If op=0 is set during significant speed, momentum may push the motor going over additional steps
            % Unless the op is 0, two more arguments are needed...
            % The fourth argument is the number of steps that the motor has to complete.
            % This defines forward and backwards direction in a range of -2^31 to 2^31-1 steps.
            % Keep in mind, that while rpm speed is keep constant,
            % the number of steps to go over a particular distance will change proportionally  with the mode.
            % The fifth argument is the mode, optional, which is a number specifying the motion mode.
            % Mode can be 0 to 7, this binarily maps to the 3 bit microstep modes MS3 MS2 MS1
            % in the used stepper controller. For this to work the Ramps jumpers on the SPI 2X4 black headers,
            % under the controller modules, must be placed. See more info in the stepperAttach() command.
            % For the controllers mentioned above, supported mode values are: 
            % MS3 MS2 MS1
            %  0   0   0 = 0-Full Step
            %  0   0   1 = 1-Half Step
            %  0   1   0 = 2-Quarter Step
            %  0   1   1 = 3-Eighth Step
            %  1   0   0 = 4-Sixteenth Step (in DRV8825)
            %  1   0   1 = 5-Thirtysecond Step (in DRV8825)
            %  1   1   0 = 6-Thirtysecond Step (in DRV8825)
            %  1   1   1 = 7-Sixteenth Step (in A4988)
            % The sixth argument, optional, defines a pin to connect a limit switch to home (pull up and assert low (pol=0) is used by default, high with pol=1)
            % Any pin value from 2 to 69 can be used, a 0 (default if parameter not used) means regular stepping with no homing switch to be done.
            % If a 2 to 69 pin is defined, steeping will stop when the limit switch is asserted, or the step count ends, whatever happens first.
            % The seventh and last argument, optional defines the polarity of the limit swith, 0 for low 1 for high  
            %
            % NOTE 1: If mode argument is ommited, the 0 value is taken, see NOTE 3 below
            % NOTE 2: The mode setting MS2-MS1 is shared by all 5 stepper in the Ramps board, MS3 is individual. Every call use its mode setting
            % NOTE 3: To free extra pins and SPI, the modes can also can be controlled manually by moving jumper positions (hardware setting)
            % These jumpers are under the controller modules, and placing them between the white and black header will set the desired microstepping
            % But this mode parameter setting, if diferent than 0, still must be explicity set in both software or hardware modes to keep speed right.
            %
            % Examples:
            % 
            % stepperStep(r,5,8,500,1);     % Runs stepper 5 forward for 500 steps in half step mode
            % stepperStep(r,2,10,-500,2);   % Runs stepper 2 backward for 500 steps in quarter step mode with auto disable
            % r.stepperStep(2,0);           % Stops and release stepper 2
            % stepperStep(r,5,10,2000,1,25);% Runs stepper 5 forward 2000 steps in half step mode (so 5 turns) OR
                                            % stop when asserting low limit switch at pin 25. Whatever is first.
            
            %%%%%%%%%%%%%%%%%%%%%%%%% ARGUMENT CHECKING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % check arguments if r.chkp is true
            if r.chkp
                % check nargin
                if nargin>7 || nargin <3
                    error('Function must have at least 3 and no more than 7 arguments including the object');
                end
                % check stepper motor number
                errstr=rampino.checknum(motor,'stepper number',1:5);
                if ~isempty(errstr), error(errstr); end
                % check op
                errstr=rampino.checknum(op,'operation',0:11);
                if ~isempty(errstr), error(errstr); end
                % if it is not released must have all arguments
                if op>7  && nargin < 4
                    error('Number of steps are missing, cannot start moving otherwise!');
                end
                % can't move forward or backward if speed is set to zero or less
                if op>7 && r.sspd(motor,1)<1
                    error('The stepper speed has to be greater than zero for the stepper to move');
                end   
                % check limit switch pin
                if nargin>5
                    errstr=rampino.checknum(mode,'limit switch pin',0:69);
                    if ~isempty(errstr), error(errstr); end
                end
                % check microstep motion mode
                if nargin>4
                    errstr=rampino.checknum(mode,'stepper mode',0:7);
                    if ~isempty(errstr), error(errstr); end
                end
                % check number of steps
                if nargin>3 && (steps < -2^31 || steps > 2^31 -1)
                    error('Value of steps out of range, must be from -2^31 to  2^31 -1' );
                end 
            end
            if nargin<7
                  pol=0; % Polarity of limit switch is active low
            end        
            if nargin<6
                  pin=0; % No homing limit switch used
            end
            if nargin<5
                  mode=0;
            end
            if nargin<4
                  steps=0;
            end   
            if pol==1 % Active high limit switch
               pin=pin+68; %Shift pin for active high
            end
            %%%%%%%%%%%%%%%%%%%%%%%%% RUN THE STEPPER %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % send motor and op first
            write(r.aser,[68 48+motor 48+op],'char');
            if op>7 % Mode and steps only needed with running operations (op>7) 
                write(r.aser,[48+mode 97+ pin],'char');   %Microstep Mode and limit pin (0 if not used)
                write(r.aser,steps,'int32');    %step Count
            end
        end % stepperstep

 %%    %% SPI Display%% 
        % dotStarLED 
        function dotStarPixs(r,op,value)        
            % dotStarPixs(r,op,value); configures dotstar LEDS individualy or in groups
            % Control is performed by SPI hardware interface at pins MOSI and CLK 
            % The first argument, r, is the rampino object. The second argument, 
            % is op for configuration operation, and  can be:
            %   1, Enable the SPI interface to get LED chain ready
            %   2, Show (Run) the previously set pixel with dSSPixs()
            %   3, Clear Off all LEDs
            %   4, Set number of pixels (value parameter) If not used the hardware is 64 pixels long by default.
            %      The max number is limited by hardware and performance factors
            %   5  Set Brightness globaly to a 256 scale (value parameter). This runs independent of  dSSPixs() 
            % The third argument is the value, either number of pixels configured or brightness scale, in operations 4 and 5
            % Brightness is limited from 0 to 255, and Number of pixels 1 to 65535 (the max limit will be much smaller) 
            % For the other operations, ommit value.
            %
            % Examples: 
            % dotStarPixs(r,2,16);       % Set a 16 pixelsdotStar chain
            % r.dotStarPixs(2,16);       % Same as above
            % dotStarPixs(r,3);          % Show LEDs, Run last led sets, dSSPixs()            
            %%%%%%%%%%%%%%%%%%%%%%%%% ARGUMENT CHECKING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % check arguments if r.chkp is true
            if r.chkp
                % check nargin
                if nargin<2 || nargin>3
                    error('Function must have at least 2 and no more than 3 arguments including the object');
                end
                % check op
                errstr=rampino.checknum(op,'operation',1:5);
                if ~isempty(errstr), error(errstr); end
                % For ops 4 an 5  must have the value argument
                if op>3  && nargin < 3
                    error('Value is missing');
                end
                % check number of LED Pixs
                if op==4 && nargin==3 && (value < 1 || value > 65535)
                    error('Value for number of LED Pixels out of range, must be from 1 to 65535' );
                end
                % check global brightness
                if op==5 && nargin==3 && (value < 0 || value > 255)
                    error('Value for brightness is out of range, must be from 0 to 255' );
                end 
            end

            %%%%%%%%%%%%%%%%%%%%%%%%% CONFIGURE DOTSTAR LED %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % send LEDs op first
            write(r.aser,[75 48+op],'char');
            if op==4 % Send Number of pixels value 
                write(r.aser,value,'uint16');  
            elseif op==5  %Send Brightness values
                write(r.aser,value,'uint8'); 
            end
        end %dotStarLED

        % dSSPixs 
        function dSSPixs(r,red,green,blue,start,len)        
            % dSSPixs(r,red,green,blue,start,len); set dotstar LEDs colors individualy or in groups
            % LED chain must be previously enabled with dotStarLED, then after set the LED or LEDs run a show operation 
            % The first argument, r, is the rampino object. The 2nd, 3rd, and 4th argument are the intensity of the colors. 
            % The intensity range of each color goes from 0 to 255 what can create up to 2^24 different colors 
            % The 5th argument, start, is the LED position in the chain if only 1 is addresed, or the start LED if multiple
            % are addressed simultaneously. This has a range of 0 to 65535, 0 being the first LED in the chain
            % The last 6th argument, len,  is the len or number of LEDs to be addressed from the start one. This has a   
            % range of 0 to 65535. If len is ommited, only the start LED is addressed, but a len of 0 will light
            % all leds to the end of the chain
            % 
            % Examples: 
            % dSSPixs(r,255,0,0,8);            % Will light 9th LED full Red
            % r.dSSPixs(0,0,255,8);            % Same as above but full Blue
            % dSSPixs(r,255,255,255,0,0);      % WARNING this will light full intensity white. Make sure
                                               % that enough power supply and cooling is provided  
                                               
            %%%%%%%%%%%%%%%%%%%%%%%%% ARGUMENT CHECKING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % check arguments if r.chkp is true
            if r.chkp
                % check nargin
                if nargin<5 || nargin>6
                    error('Function must have either 5 or 6 arguments including the object');
                end
                % check start and lenght of LED Pixs
                if (start < 0 || start > 65535)
                    error('Value for start LED is out of range, must be from 0 to 65535' );
                end
                if ( nargin==6 && len < 0 || nargin==6 && len > 65535)
                    error('Value for LED length is out of range, must be from 0 to 65535' );
                end
                % check RGB values
                if (red < 0 || red > 255 || green < 0 || green > 255 || blue < 0 || blue > 255)
                    error('Value for R,G, or B is out of range, must be from 0 to 255' );
                end 
            end
            
            %%%%%%%%%%%%%%%%%%%%%%%%% CONFIGURE DOTSTAR LED %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % send LEDs op, then code and send first 5 arguments and if len is available  command for filling
            if nargin==6  %This has len so use filling command
                write(r.aser,[75 61 red green blue],'char');
                write(r.aser,[start len],'uint16');
            else %5 arguments (no len)
                write(r.aser,[75 60 red green blue],'char');
                write(r.aser,start,'uint16');
            end
        end %dSSLED
        
%%      %%% I2C BUS%%%%     
        %I2C Switch 
        function res=i2CSwitch(r,op,value)        
            % res=i2cSwitch(r,op,channel) enable and switch I2C buses
            % The first argument, r, is the rampino object. 
            % The 2th argument is the operation
            %   1-Enables  Switch TCA9548 with I2C address. 0x70 is default, 0x71 in Mozzy project
            %   2-Select the channel  
            % The third argument is used for address or selected channel. A selected channel value 0 to 255 is possible,
            % 0 is none, then 1, 2, 4,...128, for channels 0 to 7, a 255 will select all channels (8 channels,1 per bit).
            % value= 2^n where n is the channel. It can be summed to select multiple channels in a multibus
            % The res output argument returns false if there was no error, or true for validation communication error. 
            %
            % Examples: 
            % res=i2CSwitch(r,1);              % Enable a TCA9548 in default x70, res= false if opeation ok
            % r.i2CSwitch(1,0x71);             % Same as above but address 0x71
            % i2CSwitch(r,2,6);                % Select channel 1 and 2 simultaneusly. 2^1+2^2=6
                                                                            
            %%%%%%%%%%%%%%%%%%%%%%%%% ARGUMENT CHECKING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % check arguments if r.chkp is true
            if r.chkp
                % check nargin
                if nargin<2 || nargin>3
                    error('Function must have either 2 or 3 arguments including the object');
                end
                % check value 
                if nargin==3 
                    if (value < 0 || value > 255)
                        error('Value must be from 0 to 255' );
                    end
                end
                if (op < 1 || op > 2)
                    error('Operation must be from ENABLE-1 or SELECT-2' );
                end
            end
            if nargin==2 %No value
               if op==1
                   value=0x70;
               else
                   value=255; %All channels together 
               end
            end    
            %%%%%%%%%%%%%%%%%%%%%%%%% SEND DATA %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % send LEDs op, then code and send first 5 arguments and if len is available  command for filling
            write(r.aser,[77 op value],'uint8');
            res=read(r.aser,1,'uint8');              
        end %i2CSwitch
        
        %SFM Flow sensor Enable
        function res=sfm3000en(r,sF,off)
            % res=sfm3000en(r,sF,off) enable SFM3000 Flow sensor
            % The first argument, r, is the rampino object. 
            % The 2th argument is the Scale Factor that defaults (per SFM34000-33 sensor DS) to 800
            % The 3rd argument is the value offset that defaults (per SFM34000-33 sensor DS) to 32768
            % The res output argument returns false if there was no error, or true for validation communication error.
            %
            % Examples: 
            % err=sfm3000en(r,400,0);           % Enable a SFM3000 series with 400 Scale Factor, and no offset
            % r.sfm3000en();                    % Enable an SFM3400-33 with default spec SF and offset
                                                                            
            %%%%%%%%%%%%%%%%%%%%%%%%% ARGUMENT CHECKING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
            if r.chkp
                % check nargin
                if nargin~=1 && nargin~=3 
                    error('Function must have either 1 or 3 arguments including the object');
                end
                % check value 
                if nargin==3 
                    if (sF < 0 || sF > 65535)
                        error('Scale Factor must be from 0 to 65535' );
                    end
                    if (off < 0 || off > 65535)
                        error('Offset Factor must be from 0 to 65535' );
                    end
                end
            end
            if nargin==1 %No parameters, so use default 
               sF=800;
               off=32768;
            end    
            %%%%%%%%%%%%%%%%%%%%%%%%% SEND DATA %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            write(r.aser,[77 30],'uint8');
            write(r.aser,[sF off],'single');
            write(r.aser,0x40,'uint8');
            res=read(r.aser,1,'uint16');    
        end %sfm3000en
        
       %SFM Flow sensor read
       function val=sfm3000(r)
            % val=sfm3000(r) SFM3000 Flow sensor read
            % The single input argument, r, is the rampino object. 
            % The val output argument returns single float in lpm, or NaN for validation communication error.
            %
            % Examples: 
            % val=sfm3000(r);           % Read SFM3000 series sensor
            % myFlow=r.sfm3000();       % Same as above
                                                                            
            %%%%%%%%%%%%%%%%%%%%%%%%% ARGUMENT CHECKING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if r.chkp
                % check nargin
                if nargin~=1  
                    error('Function must have at least the object argument');
                end
            end
            %%%%%%%%%%%%%%%%%%%%%%%%% SEND DATA %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            write(r.aser,[77 31],'uint8');
            val=read(r.aser,1,'single');    
       end % val=sfm3000(r)
       
       %SHTC3 Humidity Sensor Enable
       function res=shtc3en(r)
            % res=shtc3en(r) SHTC3 Humidity Sensor Enable
            % The single input argument, r, is the rampino object. 
            % The res output argument returns false if there was no error, or true for validation communication error.
            %
            % Examples: 
            % err=shtc3en(r);           % Enable SHTC3 series sensor
            % err=r.shtc3en();          % Same as above
            
            %%%%%%%%%%%%%%%%%%%%%%%%% ARGUMENT CHECKING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if r.chkp
                % check nargin
                if nargin~=1  
                    error('Function must have at least the object argument');
                end
            end
           
            %%%%%%%%%%%%%%%%%%%%%%%%% SEND DATA %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            write(r.aser,[77 20],'uint8');
            res=read(r.aser,1,'uint8');    
       end %res=shtc3en(r)
  
       %SHTC3 Humidity Sensor Temperature reading
       function val=shtc3temp(r)
            % val=shtc3temp(r) SHTC3 Humidity Sensor Temperature reading
            % The single input argument, r, is the rampino object. 
            % The val output argument returns single float in C degrees, or NaN for validation communication error.
            %
            % Examples: 
            % val=shtc3temp(r);           % Read SHTC3 sensor temperature
            % myTemp=r.shtc3temp();       % Same as above
             
            %%%%%%%%%%%%%%%%%%%%%%%%% ARGUMENT CHECKING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if r.chkp
                % check nargin
                if nargin~=1  
                    error('Function must have at least the object argument');
                end
            end         
            %%%%%%%%%%%%%%%%%%%%%%%%% SEND DATA %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            write(r.aser,[77 22],'uint8');
            val=read(r.aser,1,'single');    
       end %val=shtc3temp(r)
       
       %SHTC3 Humidity Sensor reading
       function val=shtc3hum(r)
            % val=shtc3hum(r) SHTC3 Humidity Sensor reading
            % The single input argument, r, is the rampino object. 
            % The val output argument returns single float in 0-100% humidity, or NaN for validation communication error.
            %
            % Examples: 
            % val=shtc3hum(r);           % Read SHTC3 sensor humidity
            % humidity=r.shtc3hum();     % Same as above
             
            %%%%%%%%%%%%%%%%%%%%%%%%% ARGUMENT CHECKING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if r.chkp
                % check nargin
                if nargin~=1  
                    error('Function must have at least the object argument');
                end
            end    
            %%%%%%%%%%%%%%%%%%%%%%%%% SEND DATA %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            write(r.aser,[77 21],'uint8');
            val=read(r.aser,1,'single');    
       end %val=shtc3hum(r)
       
       %STC3 CO2 sensor Enable
       function res=stc3en(r, addr,gas)
            % res=stc3en(r, addr,gas) enable STC3 CO2 sensor
            % The first argument, r, is the rampino object. 
            % The 2th argument is the I2C address that defaults to 0x29, up to 0x2C is configurable
            % The 3rd argument is the gas type and rangemeasured. The possible values are:
            %  0 for CO2+N2 gas with 100% range
            %  1 for CO2+AIR gas with 100% range
            %  2 for CO2+N2 gas with 25% range
            %  3 for CO2+AIR gas with 25% range (default value)
            % The res output argument returns false if there was no error, or true for validation communication error.
            %
            % Examples: 
            % err=stc3en(r, 0x2C,0)      % Enable an STC3 Sensor at address 0x2C to sense CO2+N2 in a 100% range
            % r.stc3en()                 % Enable an STC3 Sensor at defaults address 0x29 to sense CO2+AIR in a 25% range
                                                                            
            %%%%%%%%%%%%%%%%%%%%%%%%% ARGUMENT CHECKING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%             
            if nargin<3, gas=3; end  %default 25% range CO2 with Air 
            if nargin<2, addr=0x29; end %default I2C address
            if r.chkp
                % check nargin
                if nargin<1 || nargin>3
                    error('Function must have either 1, 2, or 3 arguments including the object');
                end
                % check value 
                if (gas < 0 || gas > 3)
                    error('Gas Type must be from 0 to 3' );
                end
                if (addr < 0x29 || addr >0x2c)
                    error('No legal address for this sensor' );
                end
            end
            %%%%%%%%%%%%%%%%%%%%%%%%% SEND DATA %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
            write(r.aser,[77 10 addr gas],'uint8');
            res=read(r.aser,1,'uint8');    
       end %res=stc3en(r, addr,gas)
       
       %STC3 CO2 sensor Settings
       function res=stc3set(r,op,val)
            % res=stc3set(r,op,val) STC3 CO2 sensor Settings
            % The first argument, r, is the rampino object. 
            % The 2th argument is the setting operation:
            %  0 to set reference CO2 percentage in % for calibration. 0% is default (0.05% -500ppm- for air environment) 
            %  1 to set compensation temperature in C. -20 to 85C is the operation range
            %  2 to set compensation humidity percentage. If no used 0% is used
            %  3 to set compensation pressure in mBar. 600 to 1200mBar is allowed
            % The 3rd argument is the value for the operations above, as a single float for the unit and range described.
            % The res output argument returns false if there was no error, or true for validation communication error.
            %
            % Examples: 
            % err=stc3set(r,1,0.05)     % Set STC3 CO2 sensor reference 0.05% known gas value to calibrate
            % r.stc3set(2,40)           % Set STC3 CO2 sensor 40% current humidity percentage for compensation
            
            %%%%%%%%%%%%%%%%%%%%%%%%% ARGUMENT CHECKING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
            if r.chkp
                % check nargin
                if nargin~= 3
                    error('Function must have 3 arguments including the object');
                end
                % check value 
                if (op < 0 || op > 3)
                    error('Operations must be from 0 to 3' );
                else
                    if op==0 && (val<0 || val>100)
                        error('The CO2 setting can only be from 0 to 100%' );
                    elseif op==1 && (val<-20 || val>85)
                        error('The temperature setting can only be from -20C to 85C' );
                    elseif op==2 && (val<0 || val>100)
                        error('The humidity setting can only be from 0% to 100%, and won''t operate reaching the dew point');
                    elseif op==3 && (val<600 || val>1200)
                        error('The environmental pressure must be between 600mBar and 1200mBar' );
                    end
                end
            end

            %%%%%%%%%%%%%%%%%%%%%%%%% SEND DATA %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
            write(r.aser,[77 op+11],'uint8');
            write(r.aser,val,'single');
            res=read(r.aser,1,'uint8');    
       end %res=stc3set(r,op,val)
      
       %STC3 CO2 sensor reading temperature
       function val=stc3temp(r)
            % val=stc3temp(r) STC3 CO2 sensor reading temperature
            % The single input argument, r, is the rampino object. 
            % The val output argument returns single float in C degrees, or NaN for validation or no ready error.
            % Note this function needs 1 second to produce a new value, otherwise NaN is returned
            %
            % Examples: 
            % val=stc3temp(r);           % Read STC3 sensor temperature
            % myTemp=r.stc3temp();       % Same as above
                                                                            
            %%%%%%%%%%%%%%%%%%%%%%%%% ARGUMENT CHECKING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if r.chkp
                % check nargin
                if nargin~=1  
                    error('Function must have at least the object argument');
                end
            end
            %%%%%%%%%%%%%%%%%%%%%%%%% SEND DATA %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
            write(r.aser,[77 15],'uint8');
            val=read(r.aser,1,'single');    
       end %val=stc3temp(r)
       
       %STC3 CO2 sensor reading 
       function val=stc3co2(r)
            % val=stc3co2(r) STC3 CO2 sensor reading 
            % The single input argument, r, is the rampino object. 
            % The val output argument returns single float in percentage, or NaN for validation or no ready error.
            % Note this function needs 1 second to produce a new value, otherwise NaN is returned
            %
            % Examples: 
            % val=stc3co2(r);           % Read STC3 sensor CO2 percentage
            % co2=r.stc3co2();          % Same as above
                                                                            
            %%%%%%%%%%%%%%%%%%%%%%%%% ARGUMENT CHECKING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if r.chkp
                % check nargin
                if nargin~=1  
                    error('Function must have at least the object argument');
                end
            end          
           
            %%%%%%%%%%%%%%%%%%%%%%%%% SEND DATA %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
            write(r.aser,[77 16],'uint8');
            val=read(r.aser,1,'single');  
       end %val=stc3co2(r)
            
    end % methods
 
 %%   STATIC
    methods (Static) % static methods
   
        function errstr=checknum(num,description,allowed)          
            % errstr=rampino.checknum(num,description,allowed); Checks numeric argument.
            % This function checks the first argument, num, described in the string
            % given as a second argument, to make sure that it is real, scalar,
            % and that it is equal to one of the entries of the vector of allowed
            % values given as a third argument. If the check is successful then the
            % returned argument is empty, otherwise it is a string specifying
            % the type of error.
            
            % initialize error string
            errstr=[];            
            % check num for type
            if ~isnumeric(num)
                errstr=['The ' description ' must be numeric'];
                return
            end           
            % check num for size
            if numel(num)~=1
                errstr=['The ' description ' must be a scalar'];
                return
            end
            
            % check num for realness
            if ~isreal(num)
                errstr=['The ' description ' must be a real value'];
                return
            end
            % check num against allowed values
            if ~any(allowed==num)
                
                % form right error string
                if numel(allowed)==1
                    errstr=['Unallowed value for ' description ', the value must be exactly ' num2str(allowed(1))];
                elseif numel(allowed)==2
                    errstr=['Unallowed value for ' description ', the value must be either ' num2str(allowed(1)) ' or ' num2str(allowed(2))];
                elseif max(diff(allowed))==1
                    errstr=['Unallowed value for ' description ', the value must be an integer going from ' num2str(allowed(1)) ' to ' num2str(allowed(end))];
                else
                    errstr=['Unallowed value for ' description ', the value must be one of the following: ' mat2str(allowed)];
                end              
            end           
        end % checknum
        
        function errstr=checkstr(str,description,allowed)    
            % errstr=rampino.checkstr(str,description,allowed); Checks string argument.
            % This function checks the first argument, str, described in the string
            % given as a second argument, to make sure that it is a string, and that
            % its first character is equal to one of the entries in the cell of
            % allowed characters given as a third argument. If the check is successful
            % then the returned argument is empty, otherwise it is a string specifying
            % the type of error.
            
            % initialize error string
            errstr=[];           
            % check string for type
            if ~ischar(str)
                errstr=['The ' description ' argument must be a string'];
                return
            end            
            % check string for size
            if numel(str)<1
                errstr=['The ' description ' argument cannot be empty'];
                return
            end
            % check str against allowed values
            if ~any(strcmpi(str,allowed))
                % make sure this is a hozizontal vector
                allowed=allowed(:)';
                % add a comma at the end of each value
                for i=1:length(allowed)-1
                    allowed{i}=['''' allowed{i} ''', '];
                end               
                % form error string
                errstr=['Unallowed value for ' description ', the value must be either: ' allowed{1:end-1} 'or ''' allowed{end} ''''];
                return
            end        
        end % checkstr 
        
    end % static methods
end % class def