package configure

import (
	"github.com/robolaunch/robot-operator/internal/label"
	corev1 "k8s.io/api/core/v1"
)

func SchedulePod(pod *corev1.Pod, tenancy label.Tenancy) *corev1.Pod {

	tenancyMap := label.GetTenancyMapFromTenancy(tenancy)
	pod.Spec.NodeSelector = tenancyMap

	return pod
}
