FROM ubuntu:16.04

# Inform scripts that no questions should be asked
ENV DEBIAN_FRONTEND=noninteractive \
# Set some environment variables to prevent warnings and errors
    CI=true \
    LANG=C.UTF-8 \
    DOCKER=true

# Install commands used in our scripts and standard present on a clean ubuntu installation
RUN apt-get update -qq && apt-get install -qq --assume-yes --no-install-recommends sudo apt-utils git wget curl lsb-release ca-certificates apt-transport-https

# Add amigo user
RUN adduser --disabled-password --gecos "" amigo && \
    adduser amigo sudo && \
    echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER amigo
WORKDIR /home/amigo

# Remove interactive check from bashrc, otherwise bashrc refuses to execute
RUN sed -e s/return//g -i ~/.bashrc

# Setup tue env
RUN mkdir -p ~/.tue

# The source of tue-env is already checked out in the current dir, moving this to the docker
ADD ./ /home/amigo/.tue/
RUN sudo chown -R amigo:amigo /home/amigo/.tue/

# Run the standard installation script
RUN /home/amigo/.tue/installer/bootstrap.bash

# Already install ros since we will use this anyway
RUN bash -c 'source /home/amigo/.bashrc && tue-get install ros'

# Cleanup, make sure image is usable like any other computer
RUN sudo chown -R amigo:amigo ~/.tue
RUN cd ~/.tue && git remote set-url origin https://github.com/tue-robotics/tue-env.git
