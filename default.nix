{ lib
, stdenv
, stdenvNoCC
, fetchurl
, cmake
, llvmPackages

, ninja
, spdlog
, libyamlcpp
, boost
, gtest
, protobuf
, eigen
, pugixml
, range-v3
, robin-map
, zstd

, commonroad-cmake
, commonroad_io

, enableDebug ? true
, enableSanitizers ? false
, enablePython ? false
, pythonPackages
, enableDocs ? true
, doxygen
, graphviz-nox

}:

assert enablePython -> !enableSanitizers;
assert enablePython -> !enableDocs;

let
  builder =
    if enablePython
    then pythonPackages.buildPythonPackage
    else stdenv.mkDerivation;

    cppreference_tags = stdenvNoCC.mkDerivation rec {
      name = "cppreference_tags";
      version = "20190607";
      src = fetchurl {
        url = "https://upload.cppreference.com/mwiki/images/1/16/html_book_${version}.tar.xz";
        hash = "sha256-j5eyuqdJx0ii4CLXhfGi6VqoUaMHWYffzzi69l4OSG0=";
      };
      sourceRoot = ".";
      installPhase = ''
          mkdir $out
          cp cppreference-doxygen-web.tag.xml $out
        '';
    };

  cmakeFlags =
    [
      "-DFETCHCONTENT_SOURCE_DIR_COMMONROAD_CMAKE=${commonroad-cmake}"
      "-DFETCHCONTENT_SOURCE_DIR_CPPREFERENCE_TAGS=${cppreference_tags}"

      # "-DCMAKE_DISABLE_PRECOMPILE_HEADERS=ON"
    ]
    ++ lib.optional enableSanitizers "-DCOMMONROAD_SANITIZERS=ASAN_UBSAN";

  version = "2022.0";
in
builder
  (
    rec {
      pname = "environment-model";
      inherit version;

      src = lib.sources.cleanSource ./.;

      outputs = [ "out" ] ++ lib.optional enableDocs "doc";

      nativeBuildInputs =
        [
          cmake
        ]
        ++ lib.optionals enableDocs [
          doxygen
          graphviz-nox
        ]
        ++ lib.optionals enablePython (with pythonPackages; [
          ninja
          scikit-build
          setuptools-scm
          pybind11
        ]);

      buildInputs = [
        protobuf
        pugixml
      ] ++ lib.optional stdenv.cc.isClang llvmPackages.openmp;

      checkInputs = [
        gtest
      ];

      preBuild = ''
        make doc_doxygen
      '';

      postInstall = ''
        mkdir $doc
        mv doc_doxygen $doc
      '';

      inherit cmakeFlags;

      passthru.cmakeFetchContentFlags = cmakeFlags;

      propagatedBuildInputs = [
        spdlog
        boost
        eigen
        libyamlcpp
        range-v3
        robin-map
      ];

      meta = {
        homepage = "https://commonroad.in.tum.de";
        description = ''
          Classes and methods to represent the CommonRoad format in C++17
        '';
        # license = lib.licenses.mit;
        platforms = with lib.platforms; linux ++ darwin;
      };
    } // lib.optionalAttrs (!enablePython) {
      doCheck = true;
      preCheck = lib.optional enableSanitizers "export ASAN_OPTIONS=detect_leaks=0";

      checkPhase =
        let
          disabledTests = lib.optionals (!enablePython) [
            "InterfacesTest.SamePredecessors"
            "InterfacesTest.SameRoadNetwork"
            "InterfacesTest.SameCrossings"
          ];
        in
        ''
          runHook preCheck

          ctest -E "${lib.concatMapStringsSep "|" (name: "(${lib.escape ["."] name})") disabledTests}"

          runHook postCheck
        '';

      postBuild = ''
        make doc_doxygen
        '';

      cmakeBuildType =
        if enableDebug
        then "Debug"
        else "RelWithDebInfo";
      # separateDebugInfo = true;

    } // lib.optionalAttrs enablePython (with pythonPackages; {
      dontUseCmakeConfigure = true;

      format = "pyproject";

      SETUPTOOLS_SCM_PRETEND_VERSION = version;

      SKBUILD_CONFIGURE_OPTIONS = cmakeFlags;

      nativeCheckInputs = [
        pytestCheckHook
      ];

      checkInputs =
        [
          numpy
          joblib
        ]
        ++ [
          commonroad_io
        ];

      pythonImportsCheck = [ "crcpp" ];

      pytestFlagsArray = [
        "tests/"
        "--ignore=scripts/scenario_test.py"
      ];
    })
  )