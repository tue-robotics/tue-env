FROM ubuntu:16.04

# Inform scripts that no questions should be asked
ENV DEBIAN_FRONTEND=noninteractive \
# Set some environment variables to prevent warnings and errors
    CI=true \
    LANG=C.UTF-8 \
    DOCKER=true \
    USER=amigo

# Install commands used in our scripts and standard present on a clean ubuntu installation
RUN apt-get update -qq && apt-get install -qq --assume-yes --no-install-recommends sudo apt-utils git wget curl lsb-release ca-certificates apt-transport-https

# Add amigo user
RUN adduser --disabled-password --gecos "" $USER && \
    adduser $USER sudo && \
    echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER "$USER"
WORKDIR /home/"$USER"

# The source of tue-env is already checked out in the current dir, moving this to the docker
COPY / ./.tue/

SHELL ["/bin/bash", "-c"]
RUN sudo chown -R $USER:$USER ~/.tue/ && \
    # Remove interactive check from bashrc, otherwise bashrc refuses to execute
    sed -e s/return//g -i ~/.bashrc && \
    # Run the standard installation script
    ~/.tue/installer/bootstrap.bash && \
    # Already install ros since we will use this anyway
    source ~/.bashrc && \
    tue-get install ros && \
    # Show ownership of .tue
    namei -l ~/.tue && \
    # Check git remote origin
    git -C ~/.tue remote -v
