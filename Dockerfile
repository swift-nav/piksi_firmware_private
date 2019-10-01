FROM 571934480752.dkr.ecr.us-west-2.amazonaws.com/swift-build:2019-01-07.3

# This Dockerfile contains anything that may change frequently.
# The base swift-build image is managed by Dockerfile in swift-nav/docker-recipes repo.
#
# See https://github.com/swift-nav/docker-recipes/blob/master/README.md on how to
# build and publish updates to that base image.

WORKDIR /mnt/workspace

