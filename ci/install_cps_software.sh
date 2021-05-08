# Allow permissions to all repositories
git config --global url."https://gitlab-ci-token:${CI_JOB_TOKEN}@gitlab.lrz.de/".insteadOf "git@gitlab.lrz.de:"

# curvilinear-coordinate-system
git clone --branch master git@gitlab.lrz.de:cps/commonroad-drivability-checker.git "$HOME"/commonroad-drivability-checker
cd "$HOME"/commonroad-drivability-checker/ || exit
bash build.sh -e /root/anaconda3/envs/commonroad -v 3.8 --cgal --serializer -i -j 6
cd build/lib/commonroad_dc/
mv libcrccosy.a ../../../
cd ../../../
cd "$CI_PROJECT_DIR" || exit
