FROM ubuntu:16.04

# Install commands used in our scripts
ENV DEBIAN_FRONTEND=noninteractive #Prevents apt errors that use a 'gui'
RUN apt-get update -qq && apt-get install -qq --assume-yes --no-install-recommends sudo apt-utils git wget curl lsb-release

# Add amigo user
RUN adduser --disabled-password --gecos "" amigo
RUN adduser amigo sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER amigo
WORKDIR /home/amigo

# Remove interactive check from bashrc
RUN sed -e s/return//g -i ~/.bashrc

# Setup tue env
RUN mkdir -p ~/.tue
ADD ./ /home/amigo/.tue/
RUN sudo chown -R amigo:amigo /home/amigo/.tue/
ENV CI=true
ENV LANG=C.UTF-8
RUN  /home/amigo/.tue/installer/scripts/bootstrap-ros-kinetic

# Already install ros since we will use this anyway
RUN bash -c 'source /home/amigo/.bashrc && tue-get install ros'
