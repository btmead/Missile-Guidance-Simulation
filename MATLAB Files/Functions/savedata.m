function check = savedata(name, results, figure, runID)
%savedata saves data to specified file location from all codes
    
    name = char(name);

    functions_folder = fileparts(mfilename('fullpath'));
    matlab_folder = fileparts(functions_folder);
    output_folder = fullfile(matlab_folder, 'Results', name);

    results_name = runID + ' Results.mat';
    figure_name = runID + ' Figure.fig';

    if ~exist(output_folder, 'dir')
        mkdir(output_folder)
    end

    save(fullfile(output_folder, results_name), "results");
    savefig(figure, fullfile(output_folder, figure_name));

    if isfile(fullfile(output_folder, results_name)) && isfile(fullfile(output_folder, figure_name))
        check = true;
    else
        check = false;
    end
end
