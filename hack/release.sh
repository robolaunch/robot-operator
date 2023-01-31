#!/bin/bash

if [ -z "$1" ]
  then
    echo "No version is supplied!"
    exit 1
fi

make docker-buildx docker-push gh-extract IMG=robolaunchio/robot-controller-manager:$1
make gh-select-node LABEL_KEY="robolaunch.io/organization" LABEL_VAL="robolaunch"
make gh-select-node LABEL_KEY="robolaunch.io/team" LABEL_VAL="robotics"
make gh-select-node LABEL_KEY="robolaunch.io/region" LABEL_VAL="europe-east"
make gh-select-node LABEL_KEY="robolaunch.io/cloud-instance" LABEL_VAL="cluster"
make gh-select-node LABEL_KEY="robolaunch.io/cloud-instance-alias" LABEL_VAL="cluster-alias"