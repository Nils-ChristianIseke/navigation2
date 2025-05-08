# This dockerfile can be configured via --build-arg
# Build context must be the /navigation2 root folder for COPY.
# Example build command:
# export UNDERLAY_MIXINS="debug ccache lld"
# export OVERLAY_MIXINS="debug ccache coverage-gcc lld"
# docker build -t nav2:latest \
#   --build-arg UNDERLAY_MIXINS \
#   --build-arg OVERLAY_MIXINS ./
ARG FROM_IMAGE=ros:rolling
ARG UNDERLAY_WS=/opt/underlay_ws
ARG OVERLAY_WS=/opt/overlay_ws

# multi-stage for caching
FROM $FROM_IMAGE AS cacher

# clone underlay source
ARG UNDERLAY_WS
WORKDIR $UNDERLAY_WS/src
COPY ./tools/underlay.repos ../
RUN vcs import ./ < ../underlay.repos