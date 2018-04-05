# Model training and freezing
To train the model, we followed the procedure from https://github.com/coldKnight/TrafficLight_Detection-TensorFlowAPI
## Running the AMI
The model was trained on AWS on a p2.xlarge instance, since training on the local CPU was very slow and training with a local GeForce 840m with 2 GB was not possible (resource exhausted).
We used the Udacity AMI:

udacity-carnd-advanced-deep-learning - ami-854464e0

When launching the AMI a storage size of 100 GiB was selected, since the default of 40 was not enough.

To connect with the AMI we used Putty and WinScp.
## Install Object Detection API
### Install tensorflow-gpu
```
pip install tensorflow-gpu==1.3
```
### Install protobuf compiler 
```
sudo apt-get update
sudo -H apt-get install protobuf-compiler
```
### Download tensorflow/models
```
cd tensorflow
git clone https://github.com/tensorflow/models.git
```
The model must be trained with branch 'r1.5' of the tensorflow/models.
```
cd models
git checkout r1.5
```
### Compile protbufs
```
cd research
protoc object_detection/protos/*.proto --python_out=.
export PYTHONPATH=$PYTHONPATH:`pwd`:`pwd`/slim
```
## Download models
```
wget http://download.tensorflow.org/models/object_detection/ssd_mobilenet_v1_coco_11_06_2017.tar.gz
tar -xvzf ssd_mobilenet_v1_coco_11_06_2017.tar.gz
wget http://download.tensorflow.org/models/object_detection/ssd_inception_v2_coco_11_06_2017.tar.gz
tar -xvzf ssd_inception_v2_coco_11_06_2017.tar.gz
``` 
## Download Configuration
```
cd ~
git clone https://github.com/coldKnight/TrafficLight_Detection-TensorFlowAPI
```
Copy Configs zu tensorflow/models/research
```
cd TrafficLight_Detection-TensorFlowAPI/
cp -r config ../tensorflow/models/research/
cp label_map.pbtxt ../tensorflow/models/research/
```
## Download dataset
For downloading the data set from google drive, we used the script from https://stackoverflow.com/questions/25010369/wget-curl-large-file-from-google-drive
```
pip install requests
cd ~/tensorflow/models/research/
python google_drive.py 0B-Eiyn-CUQtxdUZWMkFfQzdObUE data.zip
unzip data.zip
```
## Train Model
```
cd ~/tensorflow/models/research/
e.g.
python object_detection/train.py --pipeline_config_path=config/ssd_mobilenet-traffic-udacity_sim.config --train_dir=data/sim_training_data/sim_data_capture
```
## Freeze Model
To be able to run the model with tensorflow==1.3 (Carla), the model must be freezed using an older version of the tensorflow/models:
```
cd ~/tensorflow/models
git checkout 9a811d95c478b062393debd1559df61490b97149
```
Freeze:
e.g.
```
python object_detection/export_inference_graph.py --pipeline_config_path=config/ssd_mobilenet-traffic-udacity_sim.config --trained_checkpoint_prefix=data/sim_training_data/sim_data_capture/model.ckpt-5000 --output_directory=frozen_models/frozen_sim_mobile/
```
