%% Enter angles of hardstops by hand
H_range = [-56; 38.5];
K_range = [-137.5; 0];

%% Find max an min from voltage vector
% rows = time, columns = [HR, HL, KR, KL, GCR, GCL]

HR_vrange = [min(voltages(:,1)); max(voltages(:,1))];
HL_vrange = [min(voltages(:,2)); max(voltages(:,2))];
KR_vrange = [min(voltages(:,3)); max(voltages(:,3))];
KL_vrange = [min(voltages(:,4)); max(voltages(:,4))];

save("voltages");