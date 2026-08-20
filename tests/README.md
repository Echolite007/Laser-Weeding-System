# Unit tests

Run everything from the repository root:

```matlab
addpath('tests');
runTests            % or: runTests('Coverage') to also write tests/coverage.xml
```

`runTests` builds a `matlab.unittest` suite from this folder. The
`'Coverage'` option attaches `CodeCoveragePlugin` and writes a Cobertura
report to `tests/coverage.xml`.

## Coverage status of the project

Before this suite there were no automated tests, so every module was at 0 %
coverage. The suite targets the deterministic computation modules, i.e. the
ones that take plain structs in and return plain structs out:

| Module | Covered by | Notes |
| --- | --- | --- |
| `parameters.m` | `ParametersTest` | derived quantities and placeholder fields |
| `nominal_plant_sizing.m` | `NominalPlantSizingTest` | voltage-budget sizing relations, stroke/offset checks |
| `controller_design.m` | `ControllerDesignTest` | lead ratio, both crossover approaches, error constants |
| `discretisation.m` | `DiscretisationTest` | ZOH/Tustin discretisation, phase-margin loss, lead retuning |

Still uncovered, and why:

| Module | Reason |
| --- | --- |
| `main.m` | top-level script that wires the modules together |
| `reference.m` | builds and simulates a Simulink harness around `ReferenceGenerator_2023a.slx` |
| `simulation.m` | requires the SPACAR toolbox and a converged multibody run |
| `frequency_response_initialization.m` | script that only assigns experiment settings |
| `frequency_response_identification.m` | post-processing script that expects measurement data (`simout`) in the base workspace |
| `stability_margins_real_plant.m`, `comparison.m`, `test.m` | ad-hoc analysis scripts that depend on measured `.mat` data in the base workspace |
| `spacar/` | third-party toolbox |

The tests avoid golden numbers where the module output is a design formula;
they assert the relations between inputs and outputs instead, so a change in
a design choice (e.g. `r_arm`, `beta`) does not produce false failures while a
change in the underlying physics does.

Figures are suppressed with `DefaultFigureVisible` during the tests, and
`discretisation.m`'s summary table is captured with `evalc`, so a full run is
silent apart from the test output.
