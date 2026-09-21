% --- Extract logged signals from logsout ---

t = out.logsout.get('xpos_chrono').Values.Time;          % time vector
x_chrono = out.logsout.get('xpos_chrono').Values.Data;          % FMU body position
x_simscape = out.logsout.get('xpos_simscape').Values.Data;    % Simscape body position
spring_force = out.logsout.get('spring_force').Values.Data;     % FMU spring force output
applied_force = out.logsout.get('applied_force').Values.Data;   % the sinusoidal force in Simscape


T = table(t, applied_force, x_chrono, x_simscape, spring_force, ...
    'VariableNames', {'t','applied_force','x_chrono','x_simscape','spring_force'});

% --- Write to CSV ---
file_directory = "../results/";
writetable(T, file_directory + 'cosim_results.csv');