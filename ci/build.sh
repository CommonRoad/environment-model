#!/bin/bash

version=<version>
gitlab_img=gitlab.lrz.de:5005/maierhofer/environment-model/ci:$version

docker login gitlab.lrz.de:5005 \
&& docker build -t $gitlab_img .\
&& docker push "$gitlab_img"
