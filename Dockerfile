FROM 571934480752.dkr.ecr.us-west-2.amazonaws.com/swift-build:2019-01-07.3

# This Dockerfile contains anything that may change frequently.
# The base swift-build image is managed by Dockerfile in swift-nav/docker-recipes repo.
#
# See https://github.com/swift-nav/docker-recipes/blob/master/README.md on how to
# build and publish updates to that base image.

WORKDIR /mnt/workspace

USER root

ARG UID=1000
# Add a "dockerdev" user with sudo capabilities
# 1000 is the first user ID issued on Ubuntu; might
# be different for Mac users. Might need to add more.
RUN \
    useradd -u ${UID} -ms /bin/bash -G sudo dockerdev \
    && echo '%sudo ALL=(ALL) NOPASSWD:ALL' >>/etc/sudoers

# Make sure that home directory has right owner
COPY "docker-entrypoint.bash" /usr/local/bin
ENTRYPOINT ["/usr/local/bin/docker-entrypoint.bash"]

# This package is only needed for the HITL upload script, not for development
RUN pip3 install requests

USER dockerdev

CMD ["/bin/bash"]
