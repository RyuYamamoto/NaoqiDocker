FROM ros:kinetic

ENV DEBIAN_FRONTEND noninteractive
ENV USER root

RUN apt-get update
RUN apt-get install -y tmux
RUN apt-get install -y ipython
RUN apt-get install -y python
RUN	apt-get install -y git tmux wget tar vim
RUN	apt-get install -y ipython python python-pip
RUN apt-get install -y libeigen3-dev
RUN apt-get install -y libboost-all-dev

RUN pip install qibuild

RUN mkdir -p /root/git/
RUN git clone https://github.com/RyuYamamoto/haze_setting /root/git/haze_setting/
RUN cp /root/git/haze_setting/tmux/tmux.conf ~/.tmux.conf

RUN wget https://developer.softbankrobotics.com/download-tmp/pynaoqi-python2.7-2.5.5.5-linux64.tar.gz
RUN tar -xvzf pynaoqi-python2.7-2.5.5.5-linux64.tar.gz -C /root/
RUN rm pynaoqi-python2.7-2.5.5.5-linux64.tar.gz
RUN echo "export PYTHONPATH=${PYTHONPATH}:/root/pynaoqi-python2.7-2.5.5.5-linux64/lib/python2.7/site-packages" >> ~/.bashrc
RUN echo "export DYLD_LIBRARY_PATH=${DYLD_LIBRARY_PATH}:/root/pynaoqi-python2.7-2.5.5.5-linux64/lib" >> ~/.bashrc
RUN echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
