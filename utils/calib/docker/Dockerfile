FROM koide3/direct_visual_lidar_calibration:humble
RUN apt update && apt upgrade -y
RUN apt install python3 python3-pip -y
RUN pip3 install numpy opencv-python torch matplotlib
RUN git clone https://github.com/magicleap/SuperGluePretrainedNetwork.git
RUN echo 'export PYTHONPATH=$PYTHONPATH:./SuperGluePretrainedNetwork' >> ~/.bashrc
ENV PYTHONPATH=${PYTHONPATH:-""}:./SuperGluePretrainedNetwork
