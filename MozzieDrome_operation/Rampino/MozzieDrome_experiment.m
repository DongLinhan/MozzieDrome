folder %This is the code for the experimental control of the MozzieDrome
%circadian host-seeking assay. 
%Version 8/12/2023
%use in conjuction with mozzieFunctions

%need to add: reset flow or cap flow to 100, switch back port to 2

%This code is written to call gasline, light and
% camera
%actions at certain times. The basic idea is that we will examine at the
%beginning of every minute whether it is the minute for an action to be
%executed. 


%A TRIAL is defined as the release of CO2 and Air through the mix and smell
%chamber. A trial is always prefaced with ACCLIMATION and followed by
%FLUSH. At the hour of trial, acclimation will start first at the first
%minute of the hour, and then trial and then flush. 

diary experiment0906lvp


f = mozzieFunctions('COM3'); % The class for all actions


%----------EXPERIMENTAL PARAMETERS----------%

experimentDuration = 5; %The lengths of experiment in days
letThereBeLight = 2; %The light program: days of LD condition
sunrise = 8; % The hour in 24hr time, default 8
sunset = 20; % default 20
duskDuration = 30; % The length of light transition in minutes, default 60, can be changed 12/22/2022!
dayOneTrials = [2, 5, 8, 11, 14, 17, 20, 23]; % at what hours is a trial conducted on day one
dayTwoTrials = [2, 5, 8, 11, 14, 17, 20, 23]; % for day two
dayThreeTrials = [2, 5, 8, 11, 14, 17, 20, 23]; % for day three
dayFourTrials = [2, 5, 8, 11, 14, 17, 20, 23]; 
dayFiveTrials = [2, 5, 8]; 
startingMinute = 2; % The minute of the hour when experiment starts
acclimationSec = 0; % The length of acclimation in seconds 
trialSec = 30; % The length of the TRIAL in seconds
flushSec = 35; % The length of the flushing in seconds


%----------FIXED TIMING PARAMETERS DO NOT CHANGE----------%

secondCounter = -1; %This is to store the second
minuteCounter = -1; % this is to store the minute
dayCounter = 0; % to count days of experiment, default 0
currentDay = day(datetime ('now')); % the start of experiment
durationCounter = 0;
acclimationDurationSec = acclimationSec;
trialDurationSec = acclimationSec + trialSec;
flushDurationSec = acclimationSec + trialSec + flushSec;

%----------GAS CONTROL PARAMETERS----------%

setFlow = 1.5; %LPM
initialFlow = 10;
flow = initialFlow;

%----------LIGHT CONTROL PARAMETERS----------%

isTopIRused = false;
isTopWhiteused = true;
isTopRedused = false;


%----------DOING SOME BASIC CHECKS----------%

%{
f.switchPort (1) % Left port activated
f.adjustFlush (50) % flush gasline
disp ("Left port is activated, air is flushing through left port, check if air is coming through, and press any key to continue")
f.adjustFlush (50) % flush gasline
f.adjustFlush (50) % flush gasline
f.adjustFlush (50) % flush gasline
f.adjustFlush (50) % flush gasline
f.adjustFlush (50) % flush gasline
f.adjustFlush (50) % flush gasline
f.adjustFlush (50) % flush gasline
f.adjustFlush (50) % flush gasline
f.adjustFlush (50) % flush gasline
pause; % press any key to proceed
f.switchPort (2)
f.adjustFlush (50) % flush gasline
disp ("Right port is activated, air is flushing through right port, check if air is coming through, and press any key to continue")
f.adjustFlush (50) % flush gasline
f.adjustFlush (50) % flush gasline
f.adjustFlush (50) % flush gasline
f.adjustFlush (50) % flush gasline
f.adjustFlush (50) % flush gasline
f.adjustFlush (50) % flush gasline
f.adjustFlush (50) % flush gasline
f.adjustFlush (50) % flush gasline
f.adjustFlush (50) % flush gasline
pause;
%}



f.setupI2C
f.setupFlowMeter
%f.setupCO2Meter


if isTopIRused == true 

    f.adjustIRLight (255)

else 

    f.adjustIRLight (0)
    disp ("Top IR is diabled, make sure other IR light source is provided")

end     


f.autopilotLight (sunrise, sunset, duskDuration, dayCounter, letThereBeLight);

%----------NOW WE ARE CALLING THE FUNCTIONS----------%

while dayCounter <= experimentDuration

    %---CHECK IF IT'S EXPERIMENT TIME------%

    if currentDay ~= day(datetime ('now'))
        currentDay = day(datetime ('now'));
        dayCounter = dayCounter + 1;
    end 
    
    if (dayCounter == 1 && ismember (hour (datetime('now')), dayOneTrials)) || (dayCounter == 2 && ismember (hour (datetime('now')), dayTwoTrials)) || (dayCounter == 3 && ismember (hour (datetime('now')), dayThreeTrials))|| (dayCounter == 4 && ismember (hour (datetime('now')), dayFourTrials)) || (dayCounter == 5 && ismember (hour (datetime('now')), dayFiveTrials))

     
        %If it is experiment hour% 

        if minute(datetime('now')) == startingMinute; %acclimationStart
            
            disp ([datestr(now), newline 'THIS IS A TRIAL!!!!']);
            %port = randi ([1,2], 1) % randomize port
            %f.switchPort (2); % (port, true/false) choose
            %f.switchAir (1);
          
            while durationCounter <= acclimationDurationSec

                             
                if secondCounter ~= int16(second(datetime('now')))
                    f.autopilotLight (sunrise, sunset, duskDuration, dayCounter, letThereBeLight);
                    secondCounter = int16(second(datetime('now')));
                    durationCounter = durationCounter +1;
                    while secondCounter == int16(second(datetime('now')))
                        flowReading = f.readFlow
                        if abs(flowReading - setFlow) < 0.1
                            f.adjustFlush(flow)
                        else
                           
                            if flowReading - setFlow > 0.1
                                flow = flow -1
                                f.adjustFlush(flow)
                            else
                                flow = flow +1
                                f.adjustFlush(flow)
                            end
                        end
                    end
                end

            end


            while durationCounter <= trialDurationSec
                if secondCounter ~= int16(second(datetime('now')))
                    f.autopilotLight (sunrise, sunset, duskDuration, dayCounter, letThereBeLight);
                    secondCounter = int16(second(datetime('now')));
                    durationCounter = durationCounter +1;
                    while secondCounter == int16(second(datetime('now')))
                        flowReading = f.readFlow
                        
    
                        if abs(flowReading - setFlow) < 0.1
                            f.adjustCO2(flow)
                        else
                           
                            if flowReading - setFlow > 0.1
                                flow = flow -1
                                f.adjustCO2(flow)
                            else
                                flow = flow +1
                                f.adjustCO2(flow)
                            end
                        end
                    end
                end
            end

            while durationCounter <= flushDurationSec

                             
                if secondCounter ~= int16(second(datetime('now')))
                    f.autopilotLight (sunrise, sunset, duskDuration, dayCounter, letThereBeLight);
                    secondCounter = int16(second(datetime('now')));
                    durationCounter = durationCounter +1;
                    while secondCounter == int16(second(datetime('now')))
                        %flowReading = f.readFlow
                        
    
                        if abs(flowReading - setFlow) < 0.1
                            %f.adjustFlush(flow)
                        else
                           
                            if flowReading - setFlow > 0.1
                                %flow = flow -1
                                %f.adjustFlush(flow)
                            else
                                %flow = flow +1
                                %f.adjustFlush(flow)
                            end
                        end
                    end
                end

            end

            durationCounter = 0

                f.switchPort (2);
                f.switchAir (2);
        else

            if minuteCounter ~= minute(datetime('now'))
                f.autopilotLight (sunrise, sunset, duskDuration, dayCounter, letThereBeLight);
                minuteCounter = int16(minute(datetime('now')))
            end


        end


    elseif minuteCounter ~= minute(datetime('now'))
            f.autopilotLight (sunrise, sunset, duskDuration, dayCounter, letThereBeLight);
            minuteCounter = int16(minute(datetime('now')))
    end
end 










%{
       
    if minuteCounter ~= minute(datetime('now')) %so that we loop every minute
        
        disp ([datestr(now), newline 'it is a new minute']); 
        
        %----------LIGHT CONTROL----------%
        
        if hour(datetime('now')) == sunrise %determine if it's sunrise time
            lightIntensity = minute(datetime('now')) / duskDuration * 255; % if true: change light intensity
            disp ([datestr(now), newline 'It is sunrise time!'])
            f.adjustLight (lightIntensity, true) % (intensity, true/false)
        else 
        end
        
        
        if hour (datetime('now')) == sunset %determin if it's sunset time 
            lightIntensity = (1 - minute(datetime('now')) / duskDuration) * 255; % if true: change light intensity
            disp ([datestr(now), newline 'It is sunset time!'])
            l.adjustLight (lightIntensity, true) % (intensity, true/false)
        else 
        end
        
        %----------GAS AND CAMERA CONTROL----------%
        
        % DAY 1
        
        if dayCounter == 1 % determine if it's time for trials in day 1
            
            % ACCLIMATE: RELEASE AIR ONLY BY-PASSING MIX AND SMELL CHAMBER
            if ismember (hour (datetime('now')), dayOneTrials) & minute(datetime('now')) == acclimationStart
                disp ([datestr(now), newline 'THIS IS A TRIAL!!!!']);
                port = randi ([1,2], 1); % randomize port
                g.openPort (port, true); % (port, true/false) choose a port to open
                g.mixSmell (false); % (true/false) no odor, just air
                g.releaseAir (5, true); % (flowrate, true/false)
                % establish parallel pool
                parpool('local',1); % establish connection to a parallel worker "local" 
                f = parfeval(@startRecording,0, recordStart, recordEnd); % give task to local worker, see end of script
            
            % Video-recording is initiated during acclimation   
            elseif  ismember (hour (datetime('now')), dayOneTrials) & minute(datetime('now')) == recordStart
                disp ([datestr(now), newline 'Recording in progress!!!!']);
                
            % TRIAL: RELEASE CO2 WITH AIR THROUGH MIX AND SMELL CHAMBER
            elseif ismember (hour (datetime('now')), dayOneTrials) & minute(datetime('now')) == trialStart
                g.mixSmell (true); % We want smell only with CO2 at Trial time
                g.releaseCO2 (5, 5, true); % (flowrate, concentration, true/false)
            
                
            % FLUSH: RELEASE AIR ONLY BY-PASSING MIX AND SMELL CHAMBER    
            elseif ismember (hour (datetime('now')), dayOneTrials) & minute(datetime('now')) == trialEnd
                g.mixSmell (false); % (true/false) no odor, just air
                g.releaseAir (5, true); % (flowrate, true/false)
                
                
            % Stop video-recording
            elseif ismember (hour (datetime('now')), dayOneTrials) & minute(datetime('now')) == recordEnd
                disp ([datestr(now), newline 'Recording finished!!!!']);
                
            % End Flush, enter IDLE
            elseif ismember (hour (datetime('now')), dayOneTrials) & minute(datetime('now')) == flushEnd
                g.idle (true); % (true/false)
                myCluster = parcluster('local'); % These are for the reset of parallel worker 
                delete(myCluster.Jobs);
                delete(gcp('nocreate'));
            else 
            end
            
        else
        end 
        
        
        % DAY 2, NOT YET COMPLETE
        
        if dayCounter == 2
            if ismember (hour (datetime('now')), dayTwoTrials) & minute(datetim('now')) == 0
                disp ([datestr(now), newline 'THIS IS A TRIAL!!!!']);
            else 
            end 
        else 
        end
        
        % DAY 3, NOT YET COMPLETE
        
        if dayCounter == 3
            if ismember (hour (datetime('now')), dayThreeTrials) & minute(datetim('now')) == 0
                disp ([datestr(now), newline 'THIS IS A TRIAL!!!!']);
            else 
            end 
        else 
        end
        
        
        % Check day
        if currentDay ~= day(datetime ('now'))
            dayCounter = dayCounter +1 % add a day to the counter if it's a new day 
            disp ([datestr(now), newline 'it is a NEW DAY!!!'])
        else 
        end
     
        %Update minute
        minuteCounter = minute(datetime('now'));
    
    else % if it's not a new minute, no action
    end 

%}     
        