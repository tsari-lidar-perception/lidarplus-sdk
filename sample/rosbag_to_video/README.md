# **rosbag_to_video**

# **Install**:

**ffmpeg** is needed and can be installed on **Ubuntu** with:

    sudo apt install ffmpeg

**ros**:

    ROS must be installed on your computer,reference tutorials to https://blog.csdn.net/KIK9973/article/details/118755045  (for ubuntu18.04)
    sudo apt install python3-roslib python3-sensor-msgs python3-opencv 

## **Usage**:

    python rosbag_to_video.py [--fps 25] [--rate 1] [-o outputfile] [-v] [-s] [-t topic] bagfile1 [bagfile2] ...

    Converts image sequence(s) in ros bag file(s) to video file(s) with fixed frame rate using ffmpeg
    ffmpeg needs to be installed!

    --fps   Sets FPS value that is passed to ffmpeg
            Default is 25.
    -h      Displays this help.
    --ofile (-o) sets output file name.
            If no output file name (-o) is given the filename '<prefix><topic>.mp4' is used and default output codec is h264.
            Multiple image topics are supported only when -o option is _not_ used.
            ffmpeg  will guess the format according to given extension.
            Compressed and raw image messages are supported with mono8 and bgr8/rgb8/bggr8/rggb8 formats.
    --rate  (-r) You may slow down or speed up the video.
            Default is 1.0, that keeps the original speed.
    -s      Shows each and every image extracted from the rosbag file (cv_bride is needed).
    --topic (-t) Only the images from topic "topic" are used for the video output.
    -v      Verbose messages are displayed.
    --prefix (-p) set a output file name prefix othervise 'bagfile1' is used (if -o is not set).
    --start Optional start time in seconds.
    --end   Optional end time in seconds.

## **Example Output**:

keep default:

    python rosbag_to_video.py camera_and_state.bag
Then, the system will generate a camera_and_state.mp4 under the rosbag_to_video file.


   