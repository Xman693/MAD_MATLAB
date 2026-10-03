function [schedule, clocks] = get_schedule(obj, clocks) 

dt_sim = obj.SimParams.dt; 
update_rates = obj.UpdateRates; 


if ~isfield(clocks, 'radar_beam')
    clocks.radar_beam = 0;
end

if ~isfield(clocks, 'radar_measurement')
    clocks.radar_measurement = 0;
end

if ~isfield(clocks, 'fc')
    clocks.fc = 0;
end

if ~isfield(clocks, 'target_estimation')
    clocks.target_estimation = 0;
end

if ~isfield(clocks, 'uplink')
    clocks.uplink = 0;
end

if ~isfield(clocks, 'seeker_measurement')
    clocks.seeker_measurement = 0;
end

if ~isfield(clocks, 'homing_guidance')
    clocks.homing_guidance = 0;
end

if ~isfield(clocks, 'missile_autopilot')
    clocks.missile_autopilot = 0;
end


    if clocks.radar_beam >= update_rates.radar_beam
        schedule.RadarGimbleDue = true;  
        clocks.radar_beam = 0;  
    else 
        clocks.radar_beam = clocks.radar_beam + dt_sim; 
        schedule.RadarGimbleDue = false;  
         
    end 
 
 
    if clocks.radar_measurement >= update_rates.radar_measurement
        schedule.RadarProcessingDue = true;  
        clocks.radar_measurement = 0;  
    else 
        clocks.radar_measurement = clocks.radar_measurement + dt_sim; 
        schedule.RadarProcessingDue = false;  
 
 
    end 
 
 
 
    if clocks.fc >= update_rates.fc
        schedule.GroundProcessingDue = true;  
        clocks.fc = 0;  
    else 
        clocks.fc = clocks.fc + dt_sim; 
        schedule.GroundProcessingDue = false;  
    end 
 
    if clocks.target_estimation >= update_rates.target_estimation
        schedule.RadarTargetStateEstDue = true;  
        clocks.target_estimation = 0;  
    else 
        clocks.target_estimation = clocks.target_estimation + dt_sim; 
        schedule.RadarTargetStateEstDue = false;  
    end 
 
 
 %{
    if clocks.uplink >= update_rates.uplink
        schedule.UplinkDue = true;
        clocks.uplink = 0;
    else
        clocks.uplink = clocks.uplink + dt_sim;
        schedule.UplinkDue = false;
    end


    if clocks.seeker_measurement >= update_rates.seeker_measurement
        schedule.SeekerMeasurementDue = true;
        clocks.seeker_measurement = 0;
    else
        clocks.seeker_measurement = clocks.seeker_measurement + dt_sim;
        schedule.SeekerMeasurementDue = false;
    end


    if clocks.homing_guidance >= update_rates.homing_guidance
        schedule.HomingGuidance = true;
        clocks.homing_guidance = 0;
    else
        clocks.homing_guidance = clocks.homing_guidance + dt_sim;
        schedule.HomingGuidance = false;
    end


    if clocks.missile_autopilot >= update_rates.missile_autopilot
        schedule.MissileAutoPilot = true;
        clocks.missile_autopilot = 0;
    else
        clocks.missile_autopilot = clocks.missile_autopilot + dt_sim;
        schedule.MissileAutoPilot = false;
    end
 
 %}
    
 
 
    % 
end