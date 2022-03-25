# Bosch - Age and Gender Prediction

Given an input image or video, our model predicts the age (range) and gender for the identified people in each frame. First, it uses FASTER-RCNNs for body detection in each of the frame. Then, it makes use of ESPCN super-resolution to increase the quality of the detected persons followed by a custom model with RESNET-50 backbone to predict the age and gender attributes.

If the input is a video, it ouputs a video, a frame-by-frame image folder with each of the *bounding boxes of bodies* and *person id*. Along with it, a *predictions.csv* folder is outputed according to the prescribed format. (If the video is too large, you may choose to output only the csv folder - See Code for Instructions)

The project was developed and tested on Google Colab platform. It is recommended that GPU environment is used for faster training and prediction. The links for the original notebooks are mentioned in the following subsections.

## Instructions

Download the repository folder `Age_and_Gender_Prediction` containing the code

```
cd Age_and_Gender_Prediction
```

## Installing the dependencies

Install all the dependencies required for our code by running the following command

```
cd MP_BO_T4_CODE\Age and Gender Prediction
python -m venv bosch
cd bosch/Scripts
activate.bat
# return back to the original directory
cd ..
cd ..
```
![Screenshot 2022-03-25 222302](https://user-images.githubusercontent.com/20983723/160167592-4908a630-b93b-4c2c-95b3-a22035ee7e4d.jpg)

Install the necessary requirements in order using pip
```
pip install -r requirements.txt
pip install opencv-python
pip install -q opencv-contrib-python
pip install -q --upgrade opencv-python
pip install -q --upgrade opencv-contrib-python

```

### Prediction

To get the prediction for any video or image, simply run the `Predict.py` file  and follow the instructions mentioned in it. It is recommended to use GPU for faster prediction.

If you want to make prediction for a video,follow the below metioned steps

1) Place the video to be predicted in the `./video/` folder

2. Run the following command :
   ```
   python Predict.py --type video
   ```
3. It will ask for a filename so paste the exact file name you want prediction for .
4. The prediction will be stored in the `./prediction/{Video Name}`. It contains
5. - Outputted video with bounding boxes (if it's variables are set to True in the code. By default this is False)
   - `images` folder containing the frame by frame ID and bounding box (if it's variables are set to True in the code. By default this is False)
   - `predictions.csv` the csv file in the desired format

![Screenshot 2022-03-25 223117](https://user-images.githubusercontent.com/20983723/160167648-265115cd-d8f2-4c57-bfdd-82d29cd2a8db.jpg)
NOTE: Ignore the subsequent warning. Wait for it to process all the frames of the video (this may take some time in CPU)

If you want to make prediction for a image,follow the below metioned steps

1) Place the video to be predicted in the `./img/` folder

2. Run the following command :

   ```
   python Predict.py --type image
   ```
  
   
### Possible Errors
1. Try running the above commands in Anaconda Prompt instead of shell
2. Make sure the video/image is put in the 'video' or 'img' folder and this name of the testing video file is entered exactly when predict.py is run and "Enter the filename" prompt is asked
3. Make sure the environment is activated and the packages mentioned above are installed in it
4. If it doesn't run properly in GPU, make use of CPU
   
### Train

Create a `data` directory in the main folder and place the PETA datset according to the given format:

```
Age and Gender Prediction/
    data/
        PETA/
            images/[00001.png...19000.png]
            dataset.pkl
```

`images` folder can be found in:  [PETA Google Drive](https://drive.google.com/open?id=1q4cux17K3zNBgIrDV4FtcHJPLzXNKfYG).
`dataset.pkl` can be found in: ([dataset.pkl](https://drive.google.com/open?id=1q4cux17K3zNBgIrDV4FtcHJPLzXNKfYG).)

To train our age-gender prediction and the body detection, run the `Train.py` file by the following command.

```
python Train.py
```

## References and Description

### Github References

- https://github.com/aajinjin/Strong_Baseline_of_Pedestrian_Attribute_Recognition
- https://github.com/fannymonori/TF-ESPCN
- https://github.com/Chang-Chia-Chi/Pedestrian-Detection

### Paper References

1. Jian Jia , Houjing Huang , Wenjie Yang , Xiaotang Chen , and Kaiqi Huang:Rethinking of Pedestrian Attribute Recognition: Realistic Datasets and A Strong Baseline.

This paper proposes the PETA dataset and gives a strong baseline based on ResNet50 for Pedestrian Attribute Recognition as a multi -label learning problem. Our architecture is a result of inspiration by their strong baseline.

2. Shuai Shao , Zijian Zhao,  Boxun Li, Tete Xiao, Gang Yu, Xiangyu Zhang, Jian Sun, Megvii Inc.  : CrowdHuman: A Benchmark for Detecting Humans in a Crowd.

This paper provides a benchmark for human body detection in crowds. Baseline detectors like Faster RCNN  were tested on their annotated dataset in this paper.This paper is referred to for detecting the full body .

3. Wenzhe Shi , Jose Caballero , Ferenc Huszar´  , Johannes Totz , Andrew P. Aitken , Rob Bishop , Daniel Rueckert , Zehan Wang : Real-Time Single Image and Video Super-Resolution Using an Efficient Sub-Pixel Convolutional Neural Network

This paper uses  Efficient sub-pixel convolution network for increasing the resolution of the image in LR space unlike other deep learning models which do it in HR space. Features are extracted in LR space and super resolved to HR space without a deconvolution layer.
