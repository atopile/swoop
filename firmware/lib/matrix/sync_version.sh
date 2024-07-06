# Shallow clone PX4 matrix libary from https://github.com/PX4/PX4-Autopilot/tree/main/src/lib/matrix/matrix
# using https://stackoverflow.com/a/52269934/2988
pushd $(dirname $0)
rm -rf matrix
git clone -n --depth=1 --filter=tree:0 https://github.com/PX4/PX4-Autopilot/
pushd PX4-Autopilot
git sparse-checkout set --no-cone src/lib/matrix/matrix
git checkout
mv src/lib/matrix/matrix ../
popd
rm -rf PX4-Autopilot
popd
