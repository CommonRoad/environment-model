# Allow permissions to all repositories
git config --global url."https://gitlab-ci-token:${CI_JOB_TOKEN}@gitlab.lrz.de/".insteadOf "git@gitlab.lrz.de:"

# curvilinear-coordinate-system
git clone --branch development git@gitlab.lrz.de:cps/commonroad-curvilinear-coordinate-system.git "$HOME"/commonroad-curvilinear-coordinate-system
cd "$HOME"/commonroad-curvilinear-coordinate-system/ || exit
git checkout cpp_traffic_rule_monitor
mkdir -p build
cd build || exit
cmake -DADD_PYTHON_BINDINGS=TRUE -DPATH_TO_PYTHON_ENVIRONMENT="/opt/conda/envs/commonroad/" -DPYTHON_VERSION="3.6" -DCMAKE_BUILD_TYPE=Release ..
make
cd ..
python setup.py install
cd "$CI_PROJECT_DIR" || exit
pwd
