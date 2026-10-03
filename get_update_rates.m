function update_rates = get_update_rates()
%GET_UPDATE_RATES Update periods for the active radar and estimation path.
update_rates.radar_beam = 1/20;
update_rates.radar_measurement = 1/20;
update_rates.target_estimation = 1/100;
end
