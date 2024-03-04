package configure

import (
	"github.com/robolaunch/robot-operator/internal/label"
	"github.com/robolaunch/robot-operator/internal/node"
	"github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha2"
	corev1 "k8s.io/api/core/v1"
)

func (cfg *PodSpecConfigInjector) InjectRuntimeClass(podSpec *corev1.PodSpec, ros2Workload v1alpha2.ROS2Workload, currentNode corev1.Node) {
	if label.GetInstanceType(&ros2Workload) == label.InstanceTypeCloudInstance && node.IsK3s(currentNode) {
		nvidiaRuntimeClass := "nvidia"
		podSpec.RuntimeClassName = &nvidiaRuntimeClass
	}
}
