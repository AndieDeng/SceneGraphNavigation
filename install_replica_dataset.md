# ReplicaViewer Installation

## install brew
```
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
echo 'eval "$(/home/linuxbrew/.linuxbrew/bin/brew shellenv)"' >> /home/andie/.bashrc
eval "$(/home/linuxbrew/.linuxbrew/bin/brew shellenv)"
```
## install Pangolin dependencies
git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
./scripts/install_prerequisites.sh --dry-run recommended
./scripts/install_prerequisites.sh -m brew all
cd ..

## Install Replicate Dataset
git clone https://github.com/facebookresearch/Replica-Dataset.git
mkdir replica_v1
sudo apt-get install wget pigz unzip
git submodule update --init
sudo apt-get install cmake
sudo apt install libx11-dev
sudo apt-get install libegl1-mesa-dev
sudo apt-get install libglew-dev
./build/ReplicaSDK/ReplicaViewer ./replica_v1/apartment_0/mesh.ply ./replica_v1/apartment_0/textures/


## Install libpng and run ReplicaViewer
sudo apt-get install libjpeg-dev
sudo apt-get install libpng-dev
cd Replica-Dataset/3rdparty/Pangolin/build
cmake ..
make
cd ..
./build/ReplicaSDK/ReplicaRenderer ./replica_v1/apartment_0/mesh.ply ./replica_v1/apartment_0/textures/

## Install habitat lab
### install anaconda
```
conda create -n habitat python=3.9 cmake=3.14.0
conda activate habitat
conda install habitat-sim withbullet -c conda-forge -c aihabitat

habitat-viewer Replica-Dataset/replica_v1/apartment_0/habitat/mesh_semantic.ply 
```

## Download Replica CAD data
```
conda activate habitat
sudo apt install git-lfs
git clone git@github.com:facebookresearch/habitat-sim.git
python -m habitat_sim.utils.datasets_download --uids habitat_test_scenes --data-path data
python -m habitat_sim.utils.datasets_download --uids habitat_example_objects --data-path data
habitat-viewer /path/to/data/scene_datasets/habitat-test-scenes/skokloster-castle.glb

python -m habitat_sim.utils.datasets_download --uids replica_cad_dataset 
habitat-viewer --enable-physics --dataset data/replica_cad/replicaCAD.scene_dataset_config.json -- apt_1

python examples/example.py --scene data/scene_datasets/habitat-test-scenes/skokloster-castle.glb
```



## navigation basic
```
pip install jupyter
```

## Get RGB, Depth, and Semantic Images from Habitat in ReplicaCAD environment
```
conda activate habitat
cd habitat-sim
python examples/tutorials/nb_python/ReplicaCAD_quickstart.py
```
to get rgb, depth, semantic images in the /output folder