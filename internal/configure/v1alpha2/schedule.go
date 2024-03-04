package configure

import (
	"github.com/robolaunch/robot-operator/internal/label"
	corev1 "k8s.io/api/core/v1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
)

func (cfg *PodConfigInjector) SchedulePod(pod *corev1.Pod, obj metav1.Object) *corev1.Pod {

	pod.Spec.NodeSelector = label.GetTenancyMap(obj)

	return pod
}
