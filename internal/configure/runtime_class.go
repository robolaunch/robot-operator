package configure

import (
	"github.com/robolaunch/robot-operator/internal/label"
	"github.com/robolaunch/robot-operator/internal/node"
	"github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	corev1 "k8s.io/api/core/v1"
)

func InjectRuntimeClass(pod *corev1.Pod, robot v1alpha1.Robot, currentNode corev1.Node) *corev1.Pod {

	if label.GetInstanceType(&robot) == label.InstanceTypeCloudInstance && node.IsK3s(currentNode) {
		nvidiaRuntimeClass := "nvidia"
		pod.Spec.RuntimeClassName = &nvidiaRuntimeClass
	}

	return pod
}
