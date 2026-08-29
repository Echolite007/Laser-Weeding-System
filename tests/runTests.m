function results = runTests(varargin)
    % runTests  Run the unit test suite for the laser weeding system.
    %
    %   runTests()             runs every test in this folder.
    %   runTests('Coverage')   additionally writes a Cobertura coverage report to
    %                          tests/coverage.xml for the functions under test.
    %
    % Only the deterministic computation modules are covered; the Simulink /
    % SPACAR entry points (reference.m, simulation.m, main.m) need the models and
    % toolboxes and are therefore not part of this suite.

    thisDir = fileparts(mfilename('fullpath'));
    projectDir = fileparts(thisDir);
    addpath(projectDir);

    wantCoverage = any(strcmpi(varargin, 'Coverage'));

    suite = matlab.unittest.TestSuite.fromFolder(thisDir);
    runner = matlab.unittest.TestRunner.withTextOutput();

    if wantCoverage
        coveredFiles = fullfile(projectDir, { ...
                                             'parameters.m', ...
                                             'nominal_plant_sizing.m', ...
                                             'controller_design.m', ...
                                             'discretisation.m'});
        runner.addPlugin(matlab.unittest.plugins.CodeCoveragePlugin.forFile( ...
                                                                            coveredFiles, ...
                                                                            'Producing', matlab.unittest.plugins.codecoverage.CoberturaFormat( ...
                                                                                                                                              fullfile(thisDir, 'coverage.xml'))));
    end

    results = runner.run(suite);
    disp(table(results));
end
