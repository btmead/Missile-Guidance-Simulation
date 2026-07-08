function check = savedata(name, results, parameters, figure)
%savedata saves data to specified file location from all codes
    output_folder = fullfile(fileparts(mfilename('fullpath')), 'Results');

    name = char(name);
    results_name = [name ' Results.mat'];
    figure_name = [name ' Figure.fig'];

    if ~exist(output_folder, 'dir')
        mkdir(output_folder)
    end

    save(fullfile(output_folder, results_name), "results", "parameters");
    savefig(figure, fullfile(output_folder, figure_name));

    if isfile(fullfile(output_folder, results_name)) && isfile(fullfile(output_folder, figure_name))
        check = true;
    else
        check = false;
    end
end
