package configure

import (
	"github.com/robolaunch/robot-operator/internal/label"
	corev1 "k8s.io/api/core/v1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
)

func (cfg *PodSpecConfigInjector) SchedulePod(podSpec *corev1.PodSpec, obj metav1.Object) {
	podSpec.NodeSelector = label.GetTenancyMap(obj)
}
