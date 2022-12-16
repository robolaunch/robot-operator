# Installation
## Prerequisites

For Robot Operator v0.1.1, these prerequisites should be satisfied:

|     Tool     |       Version      |
|:------------:|:------------------:|
|  Kubernetes  |  `v1.19` and above |
| Cert-Manager | `v1.8.x` and above |
|    OpenEBS   | `v3.x.x` and above |

### Labeling Node

Select an active node from your cluster and add these labels:

```bash
kubectl label <NODE> robolaunch.io/organization=robolaunch
kubectl label <NODE> robolaunch.io/team=robotics
kubectl label <NODE> robolaunch.io/region=europe-east
kubectl label <NODE> robolaunch.io/cloud-instance=cluster
```

## Installing Robot Operator

Deploy robot operator one-file YAML using the command below:

```bash
kubectl apply -f https://github.com/robolaunch/robot-operator/releases/download/v0.1.1/robot_operator.yaml
```