# Z1 Control from Haply

sudo apt install libboost-dev libeigen3-dev build-essential cmake

mkdir ~/z1_ws
cd ~/z1_ws/
git clone --recursive git@github.com:davesarmoury/haply_z1_cpp.git
git clone --recursive git@github.com:unitreerobotics/z1_controller.git

cd z1_controller/
mkdir build
cd build/
cmake ..
make

cd ~/z1_ws/
cd haply_z1_cpp/

sudo cp udev/99-haply.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

mkdir build
cd build/
make
./haply_z1_cpp /dev/haply/inverse3_16372250 /dev/haply/handle_38DE0ED2D37F98C6 
cd ..


