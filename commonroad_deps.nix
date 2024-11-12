{ lib
, fetchFromGitLab
, fetchPypi
, python3Packages
, graphviz-nox
, enableDocs ? true
,
}: rec {
  commonroad_vehicle_models =
    python3Packages.buildPythonPackage
      rec {
        pname = "commonroad-vehicle-models";
        version = "3.0.2";

        src = fetchPypi {
          inherit pname version;
          hash = "sha256-h03jsXP3s5hptp7I2nFpD4SXZq+GI1JePpb4oz0DT44=";
        };

        propagatedBuildInputs = with python3Packages; [
          omegaconf
        ];

        checkInputs = with python3Packages; [
          scipy
          matplotlib
        ];

        nativeCheckInputs = with python3Packages; [
          pytestCheckHook
        ];

        pytestFlagsArray = [ "unit_tests/" ];

        disabledTests = [
          "test_std"
        ];
        pythonImportsCheck = [ "vehiclemodels.vehicle_dynamics_ks" ];

        meta = {
          homepage = "https://commonroad.in.tum.de";
          description = ''
            Classes and methods to represent the CommonRoad format in C++17
          '';
          license = lib.licenses.bsd3;
          platforms = with lib.platforms; linux ++ darwin;
        };
      };

  commonroad_io =
    python3Packages.buildPythonPackage
      rec {
        pname = "commonroad-io";
        version = "2022.2";

        # PyPI distribution does not include tests
        #src = fetchPypi {
        #  inherit pname version;
        #  hash = "sha256-Wu2aZo0sGHxfjeNABLfZjDFBrR0wdY1WR4rVY4r8SbI=";
        #};
        src = fetchFromGitLab {
          owner = "tum-cps";
          repo = "commonroad_io";
          domain = "gitlab.lrz.de";
          rev = "release_${version}";
          hash = "sha256-hYlZ8Dkjpc/7c+jQXjzC6hdXp9dBM5JnXSIUfKxb+WM=";
        };

        outputs = [ "out" ] ++ lib.optional enableDocs "doc";

        nativeBuildInputs = with python3Packages;
          [
            pythonRelaxDepsHook
          ]
          ++ lib.optionals enableDocs [
            sphinxHook
            graphviz-nox
          ];

        pythonRelaxDeps = [ "protobuf" ];

        propagatedBuildInputs = with python3Packages; ([
          numpy
          scipy
          shapely
          matplotlib
          lxml
          networkx
          pillow
          iso3166
          commonroad_vehicle_models
          Rtree
          tqdm
          protobuf3
        ]
        ++ lib.optionals enableDocs [
          ipython
          sphinx-autodoc-typehints
          sphinx-rtd-theme
        ]);

        passthru.optional-dependencies = with python3Packages; {
          tutorials = [ cvxpy jupyter ];
        };

        nativeCheckInputs = with python3Packages; [
          pytestCheckHook
        ];

        pytestFlagsArray = [ "tests/" ];

        disabledTests = [
          "test_open"
          "test_open_all"
          "test_open_with_lanelet_assignment"
          "test_find_lanelet_by_position"
          "test_find_lanelet_by_shape"
          "test_find_most_likely_lanelet_by_state"
          "test_assign_vehicles"
          "test_video"
        ];

        pythonImportsCheck = [ "commonroad.scenario.scenario" ];

        meta = {
          homepage = "https://commonroad.in.tum.de";
          description = ''
            Classes and methods to represent the CommonRoad format in C++17
          '';
          license = lib.licenses.bsd3;
          platforms = with lib.platforms; linux ++ darwin;
        };
      };
}
