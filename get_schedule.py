def get_schedule(obj, clocks):
    dt = obj.SimParams["dt"]
    rates = obj.UpdateRates

    for name in ("radar_beam", "radar_measurement", "target_estimation"):
        if name not in clocks:
            clocks[name] = 0

    schedule = {}
    schedule["RadarGimbleDue"], clocks["radar_beam"] = _tick(
        clocks["radar_beam"], rates["radar_beam"], dt
    )
    schedule["RadarProcessingDue"], clocks["radar_measurement"] = _tick(
        clocks["radar_measurement"], rates["radar_measurement"], dt
    )
    schedule["RadarTargetStateEstDue"], clocks["target_estimation"] = _tick(
        clocks["target_estimation"], rates["target_estimation"], dt
    )
    return schedule, clocks


def _tick(clock, interval, dt):
    due = clock >= interval
    if due:
        clock = 0
    else:
        clock += dt
    return due, clock
