package configure

import (
	"github.com/robolaunch/robot-operator/internal/label"
	"github.com/robolaunch/robot-operator/internal/node"
	corev1 "k8s.io/api/core/v1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
)

func (cfg *PodSpecConfigInjector) InjectRuntimeClass(podSpec *corev1.PodSpec, obj metav1.Object, currentNode corev1.Node) {
	if label.GetInstanceType(obj) == label.InstanceTypeCloudInstance && node.IsK3s(currentNode) {
		nvidiaRuntimeClass := "nvidia"
		podSpec.RuntimeClassName = &nvidiaRuntimeClass
	}
}
