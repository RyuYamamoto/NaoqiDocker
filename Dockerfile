FROM osrf/ros:kinetic-desktop-full

ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

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
RUN pip install almath
RUN pip install pygame

RUN mkdir -p /root/git/
RUN git clone https://github.com/RyuYamamoto/haze_setting /root/git/haze_setting/
RUN cp /root/git/haze_setting/tmux/tmux.conf ~/.tmux.conf

RUN cd /root/ && \
	git clone https://github.com/RyuYamamoto/libqi -b new_boost_version && \
	cd /root/libqi && \
	mkdir build && cd build && \
	cmake .. -DQI_WITH_TESTS=OFF && \
	make -j12 && \
	make install DESTDIR=./output

RUN cd /root/ && \
	git clone https://github.com/aldebaran/libqi-python -b team/platform/dev && \
	cd libqi-python && \
	mkdir build && cd build && \
	cmake .. -DQI_WITH_TESTS=OFF -DCMAKE_PREFIX_PATH=/root/libqi/build/output -DCMAKE_CXX_FLAGS:STRING=-std=gnu++0x && \
	make -j12 && make install && \
	cd ../ && python setup.py install

RUN wget https://developer.softbankrobotics.com/download-tmp/pynaoqi-python2.7-2.5.5.5-linux64.tar.gz
RUN tar -xvzf pynaoqi-python2.7-2.5.5.5-linux64.tar.gz -C /root/
RUN rm pynaoqi-python2.7-2.5.5.5-linux64.tar.gz
RUN echo "export PATH=/usr/lib/nvidia-396/bin:${PATH}" >> ~/.bashrc
RUN echo "export LD_LIBRARY_PATH=/usr/lib/nvidia-396:/usr/lib32/nvidia-396:${LD_LIBRARY_PATH}" >> ~/.bashrc
RUN echo "export PYTHONPATH=${PYTHONPATH}:/root/libqi-python/build/sdk/lib/python2.7/site-packages" >> ~/.bashrc
RUN echo "export DYLD_LIBRARY_PATH=${DYLD_LIBRARY}:/root/libqi-python/build/sdk/lib" >> ~/.bashrc
RUN echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
