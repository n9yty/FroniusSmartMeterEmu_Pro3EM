#!/bin/bash
#

# docker buildx build --platform linux/arm/v7,linux/arm64/v8,linux/amd64 -t n9yty/pro3emfroniusmeter:0.3 -f Dockerfile.pro3em .
docker build -t n9yty/pro3emfroniusmeter:0.3 -f Dockerfile.pro3em .
