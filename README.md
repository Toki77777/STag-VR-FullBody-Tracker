# STag-VR-FullBody-Tracker

Full-body tracking in VR using STag markers.

This is my second attempt at creating a full-body tracking system using fiducial markers. This should enable people to get fullbody tracking for free, using only a phone and some cardboard. It is possible to get pretty good tracking with trackers of sizes as small as 10cm and a PS eye camera of 640x480 resolution. Increasing the marker size or using a higher resolution and faster phone camera further improves tracking.

**NOTE: THIS IS A FREE AND OPEN SOURCE PROJECT. YOU DO NOT NEED DRIVER4VR FOR THIS!**

To use, you will have to make three trackers - one for each leg and one for hips. Using only leg trackers will not work in VRChat!

This version uses STag, a stable, occlusion-resistant fiducial marker system, and includes many improvements to make the system easier to use, such as a GUI interface and a more straightforward  calibration.

### Printing markers

The [images-to-print](images-to-print) folder contains ready-to-print A4 sheets of STag (HD11) markers, 93 mm (black square edge), sized and positioned for the default tracker setup. Each page holds one tracker's front or back marker pair, so print one front page and one back page per tracker (the front and back sheets for a given tracker carry different marker ids of that same tracker). If you need more markers, or markers of a different size, you can generate your own with [utilities/generate_stag_markers.py](utilities/generate_stag_markers.py) (requires Python with numpy, opencv-python and pillow installed).

If you have any issues or encounter any bugs, feel free to open an issue on github or message me on discord: https://discord.gg/g2ctkXB4bb

The program can be downloaded from the [releases](https://github.com/Toki77777/STag-VR-FullBody-Tracker/releases) tab.

![demo](images/demo.gif)

### Short setup video:
Coming soon

## Build instructions:

**NOTE: THIS IS ONLY FOR DEVELOPERS. IF YOU ONLY WANT TO USE APRILTAGTRACKERS AND NOT WRITE CODE, THE TUTORIAL IS ON THE [WIKI](https://github.com/Toki77777/STag-VR-FullBody-Tracker/wiki)**

The project is a CMake project. You should be able to build it either using CMake and your favourite IDE/compiler, or some IDEs already support opening cmake projects directly.

### Linux prerequisites
```
sudo apt-get update -y
sudo apt-get install -y build-essential tar curl zip unzip pkg-config autoconf libudev-dev freeglut3-dev libgtk-3-dev libsecret-1-dev libgcrypt20-dev libsystemd-dev ffmpeg
```
OpenCV gstreamer backend
```
sudo apt-get install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-doc gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio
```


### Windows prerequisites
Open in Visual Studio, or use the Visual Studio Command Prompt.


### Clone and build
```
git clone https://github.com/Toki77777/STag-VR-FullBody-Tracker
cd April-Tag-VR-FullBody-Tracker
cmake -B build
cmake --build build --config Release --target install
```

That should be it! In case you try it before a more detailed guide is up, we are always there to help on the discord server! (link above)

### Troubleshooting

If anything goes wrong when building, please notify us in the discord/via a github issue!

## Sources
Benligiray, B., Topal, C., Akinlar, C. "STag: A stable fiducial marker system." Image and Vision Computing, 2019.

https://github.com/ManfredStoiber/stag

WxWidgets: A Cross-Platform GUI Library

https://github.com/wxWidgets/wxWidgets
