%% EXPORT_SIMULINK_DIAGRAMS  Export the block diagrams of model/QuadrotorModel.slx to PNG.
% Writes the top-level diagram and every subsystem to docs/images/simulink.
%
% Usage (from the repository root):
%   matlab -batch "run('scripts/export_simulink_diagrams.m')"

projectRoot = fullfile(fileparts(mfilename('fullpath')), '..');
outDir = fullfile(projectRoot, 'docs', 'images', 'simulink');
if ~exist(outDir, 'dir'), mkdir(outDir); end

model = 'QuadrotorModel';
load_system(fullfile(projectRoot, 'model', [model '.slx']));
cleanup = onCleanup(@() close_system(model, 0));

% Skip library links (e.g. Degrees to Radians) and MATLAB Function blocks,
% whose diagrams carry no information of their own
subsystems = find_system(model, 'LookUnderMasks', 'none', 'BlockType', 'SubSystem');
isLibLink = ~cellfun(@isempty, get_param(subsystems, 'ReferenceBlock'));
isMFunc = strcmp(get_param(subsystems, 'SFBlockType'), 'MATLAB Function');
systems = [{model}; subsystems(~isLibLink & ~isMFunc)];
for k = 1:numel(systems)
    sys = systems{k};
    name = regexprep(lower(strrep(sys, [model '/'], '')), '[^a-z0-9]+', '_');
    if strcmp(sys, model), name = 'top_level'; end
    print(['-s' sys], '-dpng', '-r150', fullfile(outDir, [name '.png']));
    fprintf('%-60s -> %s.png\n', sys, name);
end
