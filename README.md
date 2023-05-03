# <img src="https://raw.githubusercontent.com/robolaunch/trademark/main/logos/svg/rocket.svg" width="40" height="40" align="top"> robolaunch Kubernetes Robot Operator

<div align="center">
  <p align="center">
    <a href="https://github.com/robolaunch/robot-operator/blob/main/LICENSE">
      <img src="https://img.shields.io/github/license/robolaunch/robot-operator" alt="license">
    </a>
    <a href="https://github.com/robolaunch/robot-operator/issues">
      <img src="https://img.shields.io/github/issues/robolaunch/robot-operator" alt="issues">
    </a>
    <a href="https://github.com/robolaunch/robot-operator/releases">
      <img src="https://img.shields.io/github/v/release/robolaunch/robot-operator" alt="release">
    </a>
  </p>
</div>

<div align="center">
  <p align="center">
    <a href="https://github.com/robolaunch/robot-operator/releases">
      <img src="https://img.shields.io/github/go-mod/go-version/robolaunch/robot-operator" alt="release">
    </a>
    <a href="https://pkg.go.dev/github.com/robolaunch/robot-operator">
      <img src="https://pkg.go.dev/badge/github.com/robolaunch/robot-operator.svg" alt="Go Reference">
    </a>
    <a href="https://goreportcard.com/report/github.com/robolaunch/robot-operator">
      <img src="https://goreportcard.com/badge/github.com/robolaunch/robot-operator" alt="Go Reference">
    </a>
  </p>
</div>

<div align="center">
  <p align="center">
    <a href="https://hub.docker.com/u/robolaunchio/robot-controller-manager">
      <img src="https://img.shields.io/docker/pulls/robolaunchio/robot-controller-manager" alt="pulls">
    </a>
    <a href="https://github.com/robolaunch/robot-operator/actions">
      <img src="https://github.com/robolaunch/robot-operator/actions/workflows/docker-build-for-push.yml/badge.svg" alt="build">
    </a>
  </p>
</div>

robolaunch Kubernetes Robot Operator manages lifecycle of ROS 2 based robots and enables defining, deploying and distributing robots declaratively.

<img src="https://raw.githubusercontent.com/robolaunch/trademark/main/repository-media/robot-operator/kubectl-get-robots.gif" alt="kubectl-get-robots" width="100%"/>

<img src="https://raw.githubusercontent.com/robolaunch/trademark/main/repository-media/robot-operator/kubectl-describe-robot.gif" alt="kubectl-describe-robot" width="100%"/>

## Table of Contents  
- [Idea](#idea)
- [Quick Start](#quick-start)
  - [Installation](#installation)
  - [Deploy Your First Robot](#deploy-your-first-robot)
- [Contributing](#contributing)


## Idea

The main idea of this project is to manage robots as Kubernetes custom resources. As a custom resource, a robot's lifecycle contains following operations and benefits.

- **Robot Lifecycle Management**
  - Deployment
  - Update
  - Upgrade
  - Vertical Scaling
    - Adjusting robot's resources
- **Robot Observability**
  - ROS observability tools (eg. rViz, Foxglove, ROS Tracker)
  - Exporting ROS nodes, topics, services, actions and bandwidth
- **GPU Acceleration**
  - Simulation (Gazebo, Ignition)
  - VDI
- **Geoghraphic Distribution**
  - Cloud-powered robot
  - Cloud-connected robot
- **Software development lifecycle**
  - Cloud IDE
- **Connectivity**
  - Robot-to-Robot Discovery
  - Node-to-Node Discovery

Refer [robolaunch.io](robolaunch.io) and [project wiki](https://github.com/robolaunch/robot-operator/wiki) for more architectural details and documentations.

## Quick Start

### Installation

Label a node in your cluster:

```bash
kubectl label <NODE> robolaunch.io/organization=robolaunch
kubectl label <NODE> robolaunch.io/team=robotics
kubectl label <NODE> robolaunch.io/region=europe-east
kubectl label <NODE> robolaunch.io/cloud-instance=cluster
kubectl label <NODE> robolaunch.io/cloud-instance-alias=cluster-alias
```

Install Robot Operator with Helm:

```bash
# add robolaunch Helm repository and update
helm repo add robolaunch https://robolaunch.github.io/charts/
helm repo update
# install chart
helm upgrade -i robot-operator robolaunch/robot-operator  \
--namespace robot-system \
--create-namespace \
--devel
```

See [installation guide for more](./docs/installation/README.md).

### Deploy Your First Robot

Robot deployment steps will be instructed here.

<!-- You can try example robots under [`config/samples/`](./config/samples/). For example, to deploy Linorobot 2, apply the YAML below.

```yaml
# linorobot2.yaml
apiVersion: robot.roboscale.io/v1alpha1
kind: Robot
metadata:
  name: linorobot2
spec:
  robot:
    nodeSelector:
      robolaunch.io/platform: "true"
    distro: foxy
    state: Launched
    tools:
      tracker:
        enabled: true
      cloudIDE:
        enabled: true
      bridge:
        enabled: false
      foxglove:
        enabled: false
      vdi:
        enabled: false
    mode: Single
    resources:
      storage: 15000
      cpuPerContainer: 800m
      memoryPerContainer: 512Mi
    namespacing: false
    workspaces:
    - name: linorobot-ws
      repositories:
      - name: master_br
        url: https://github.com/tunahanertekin/linorobot2
        branch: tuna
        launch:
          launchFilePath: linorobot2_gazebo/launch/gazebo.launch.py
          env:
          - name: LINOROBOT2_BASE
            value: 2wd
      build: Standard
```

```bash
kubectl apply -f linorobot2.yaml
```

After applying YAML, check robot's status by using:
```bash
watch "kubectl get robot linorobot2"
```

To see events and other robot-specific configurations, run:
```bash
kubectl describe robot linorobot2
``` -->


## Contributing

Please see [this guide](./CONTRIBUTING) if you want to contribute.
