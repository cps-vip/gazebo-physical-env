FROM ghcr.io/cps-vip/sabina_ros_helics_gld:latest

ARG RELEASE
ARG LAUNCHPAD_BUILD_ARCH
ARG TARGETPLATFORM=linux/arm64

LABEL org.opencontainers.image.ref.name=ubuntu
LABEL org.opencontainers.image.version=22.04
LABEL maintainer="Tiryoh<tiryoh@gmail.com>"

ADD file:4126c5ecc7750c7d2beb8c08d15aea03d96910453b36d2fb2d41185fdca7b20f in /

CMD ["/bin/bash"]

RUN apt-get update -q \
    && DEBIAN_FRONTEND=noninteractive apt-get upgrade -y \
    && apt-get autoclean \
    && apt-get autoremove \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update -q \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y ubuntu-mate-desktop \
    && apt-get autoclean \
    && apt-get autoremove \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update -q \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y \
        tigervnc-standalone-server tigervnc-common \
        supervisor wget curl gosu git sudo python3-pip tini \
        build-essential vim sudo lsb-release locales \
        bash-completion tzdata terminator dos2unix \
    && apt-get autoclean \
    && apt-get autoremove \
    && rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/AtsushiSaito/noVNC.git -b add_clipboard_support /usr/lib/novnc

RUN pip install git+https://github.com/novnc/websockify.git@v0.10.0

RUN ln -s /usr/lib/novnc/vnc.html /usr/lib/novnc/index.html

RUN sed -i "s/UI.initSetting('resize', 'off');/UI.initSetting('resize', 'remote');/g" /usr/lib/novnc/app/ui.js

RUN sed -i 's/Prompt=.*/Prompt=never/' /etc/update-manager/release-upgrades

RUN sed -i 's/enabled=1/enabled=0/g' /etc/default/apport

RUN DEBIAN_FRONTEND=noninteractive add-apt-repository ppa:mozillateam/ppa -y \
    && echo 'Package: *' > /etc/apt/preferences.d/mozilla-firefox \
    && echo 'Pin: release o=LP-PPA-mozillateam' >> /etc/apt/preferences.d/mozilla-firefox \
    && echo 'Pin-Priority: 1001' >> /etc/apt/preferences.d/mozilla-firefox \
    && apt-get update -q \
    && apt-get install -y firefox \
    && apt-get autoclean \
    && apt-get autoremove \
    && rm -rf /var/lib/apt/lists/*