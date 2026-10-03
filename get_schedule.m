function [schedule, clocks] = get_schedule(obj, clocks)
%GET_SCHEDULE Schedule radar and target-state estimation updates.

dt = obj.SimParams.dt;
rates = obj.UpdateRates;

clockNames = {'radar_beam', 'radar_measurement', 'target_estimation'};
for k = 1:numel(clockNames)
    name = clockNames{k};
    if ~isfield(clocks, name)
        clocks.(name) = 0;
    end
end

[schedule.RadarGimbleDue, clocks.radar_beam] = tick(clocks.radar_beam, rates.radar_beam, dt);
[schedule.RadarProcessingDue, clocks.radar_measurement] = tick(clocks.radar_measurement, rates.radar_measurement, dt);
[schedule.RadarTargetStateEstDue, clocks.target_estimation] = tick(clocks.target_estimation, rates.target_estimation, dt);
end

function [due, clock] = tick(clock, interval, dt)
due = clock >= interval;
if due
    clock = 0;
else
    clock = clock + dt;
end
end
