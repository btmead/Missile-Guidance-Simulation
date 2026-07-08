function [runID] = run_id(parameters)
%run_id Creates a run ID and saves parameters to a spreadsheet
%   To allow for comprehensive data logging each run will have its own
%   generated ID saved in a spreadsheet with the time it ran and the
%   parameters used. This can be used to identify results files

date_time = datetime('now');
parameter_text = jsonencode(parameters);
runID = string(abs(java.lang.String(parameter_text).hashCode));

stack = dbstack;
caller_file = stack(2).file;
[~, code_name, ~] = fileparts(caller_file);

functions_folder = fileparts(mfilename("fullpath"));
matlab_folder = fileparts(functions_folder);
log_file = fullfile(matlab_folder, "Results", "Run Log.xlsx");
sheet_name = code_name;
    
parameter_table = struct2table(parameters, "AsArray", true);
run_table = table( ...
    string(runID), date_time, ...
    'VariableNames', {'RunID', 'DateTime'});
new_row = [run_table parameter_table];

if isfile(log_file)
    try
        old_table = readtable(log_file, "Sheet", sheet_name);
        updated_table = [old_table; new_row];
    catch
        updated_table = new_row;
    end
else
    updated_table = new_row;
end

writetable(updated_table, log_file, 'Sheet', sheet_name);

end
