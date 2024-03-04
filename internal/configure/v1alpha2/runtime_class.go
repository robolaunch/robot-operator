package configure

import (
	"github.com/robolaunch/robot-operator/internal/label"
	"github.com/robolaunch/robot-operator/internal/node"
	"github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	corev1 "k8s.io/api/core/v1"
)

func (cfg *PodSpecConfigInjector) InjectRuntimeClass(podSpec *corev1.PodSpec, robot v1alpha1.Robot, currentNode corev1.Node) {
	if label.GetInstanceType(&robot) == label.InstanceTypeCloudInstance && node.IsK3s(currentNode) {
		nvidiaRuntimeClass := "nvidia"
		podSpec.RuntimeClassName = &nvidiaRuntimeClass
	}
}
