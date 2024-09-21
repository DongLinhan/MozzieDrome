classdef mozzieFunctions < rampino 
    
    properties (Constant)
        
        %Pins in use, variables about Pins are in ALL CAPS
        
        IR= 13
        WHITE= 12
        RED= 11
        AIR = 10
        CO2 = 9
        FLUSH = 8
        SWITCH = 7

        %Initial light intensities (0-255)
        
        IR_Ini= 0
        WHITE_Ini= 0
        RED_Ini= 0

    end
    
    
    
    methods
        
        function f = mozzieFunctions (com)
            f@rampino(com);
            %f.pinMode (f.IR, 'output');
            %f.pinMode (f.WHITE, 'output');
            %f.pinMode (f.RED, 'output');
            f.pinMode (f.AIR, 1)
            f.pinMode (f.CO2, 1)
            f.pinMode (f.FLUSH, 1)
            f.pinMode (f.SWITCH, 1)
            f.analogWrite (f.IR, f.IR_Ini);
            f.analogWrite (f.WHITE, f.WHITE_Ini);
            f.analogWrite (f.RED, f.RED_Ini);


        end

        function autopilotLight (f, dawn, dusk, duskDuration, dayCounter, letThereBeLight)
            
            if dayCounter > letThereBeLight
                
                f.analogWrite (f.WHITE, 0)
                disp ([datestr(now), newline 'We are under DD and at 0 intensity of WHITE light'])
                
            else 
                
                dayHours = [(dawn+1):1:(dusk-1)];
                nightHours = [(dusk+1):1:23, 0:1:(dawn-1)];
                if ismember (hour(datetime('now')), dayHours)
                    f.analogWrite (f.WHITE, 255);
                    disp ([datestr(now), newline 'We are at 255 intensity of WHITE light'])
                elseif ismember (hour(datetime('now')), nightHours)
                    f.analogWrite (f.WHITE, 0);
                    disp ([datestr(now), newline 'We are at 0 intensity of WHITE light'])
                elseif hour(datetime('now')) == dawn
                    if minute(datetime('now')) < duskDuration 
                        intensity = minute(datetime('now')) / duskDuration  * 255; % if true: change light intensity
                        intensity = int16 (intensity);
                        f.analogWrite (f.WHITE, intensity);
                        disp ([datestr(now), newline 'We are at ' num2str(intensity) ' intensity of WHITE light'])
                    else 
                        f.analogWrite (f.WHITE, 255);
                        disp ([datestr(now), newline 'We are at 255 intensity of WHITE light'])
                    end
                elseif hour(datetime('now')) == dusk 
                    if minute(datetime('now')) < duskDuration
                        intensity = (1 - minute(datetime('now')) / duskDuration)  * 255; % if true: change light intensity
                        intensity = int16 (intensity);
                        f.analogWrite (f.WHITE, intensity);
                        disp ([datestr(now), newline 'We are at ' num2str(intensity) ' intensity of WHITE light'])
                    else
                        f.analogWrite (f.WHITE, 0);
                        disp ([datestr(now), newline 'We are at 0 intensity of WHITE light'])
                    end
                end

            end
        end

        
        function adjustIRLight (f, intensity)
            f.analogWrite (f.IR, intensity)
            disp ([datestr(now), newline 'We are at ' num2str(intensity) '% intensity of IR light'])
        end 

        function adjustWhiteLight (f, intensity)
            f.analogWrite (f.WHITE, intensity)
            disp ([datestr(now), newline 'We are at ' num2str(intensity) '% intensity of WHITE light'])
        end

        function adjustRedLight (f, intensity, op)
            f.analogWrite (f.RED, intensity)
            disp ([datestr(now), newline 'We are at ' num2str(intensity) '% intensity of RED light'])
        end 
            
        function adjustAir (f, flow)
            if flow > 10
                f.analogWrite (f.AIR, 255)
                pause (100/1000) % pause for 100ms
                f.analogWrite (f.AIR, 100) % switch to holding voltage
                pause ((flow*10-100)/1000) 
                f.analogWrite (f.AIR, 0)
            else
            end

            if flow < 10
                f.analogWrite (f.AIR, 255)
                pause (flow*10/1000)
                f.analogWrite (f.AIR, 0)
            else
            end 

        end

        function switchAir (f, op)
            if op == 1
                f.analogWrite (f.FLUSH, 255)
                pause (100/1000)
                f.analogWrite (f.FLUSH, 100)
            else
                f.analogWrite (f.FLUSH, 0)
            end 

        end




        function switchPort (f, port)
            if port == 1
                f.analogWrite (f.SWITCH, 255)
                pause (100/1000)
                f.analogWrite (f.SWITCH, 100)
                pause (1)
                f.analogWrite (f.SWITCH, 80)
            end

            if port == 2
                f.analogWrite (f.SWITCH, 0)
            end
        end

        function setupI2C (f)
            i2CSwitch(f,1,0x71)
            i2CSwitch(f,2,3)
        end


        function setupFlowMeter (f)
            sfm3000en(f,800,32768)
        end 

        function val = readFlow (f)
            val = sfm3000(f);
        end 

        function setupCO2Meter (f) %only setup CO2 when there's air running!!!!!
            shtc3en(f)
            temp=shtc3temp(f)
            hum=shtc3hum(f)
            stc3en(f)
            stc3set(f,0,0.05) % 0% Ref Co2 Cal value 
            stc3set(f,1,temp) % Ref temp C Cal value 
            stc3set(f,2,hum) %  Ref humidity % Cal value 
            stc3set(f,3,1020) % Ref env. Pressure mBar Cal value 
        end

        
        function val = readCO2 (f)
            val = stc3co2(f);
        end



        function adjustFlush (f, flow)
            f.analogWrite (f.FLUSH, 255)
            pause (flow/750) % pause for 100ms
            f.analogWrite (f.FLUSH, 0)
            pause ((100-flow)/750)

        end

        function adjustCO2 (f, flow)
            f.analogWrite (f.CO2, 255)
            pause (flow/750) % pause for 100ms
            f.analogWrite (f.CO2, 0)
            pause ((100-flow)/750)

        end








    end 
end 