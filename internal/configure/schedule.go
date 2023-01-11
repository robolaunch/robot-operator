package configure

import (
	corev1 "k8s.io/api/core/v1"
)

func SchedulePod(pod *corev1.Pod, tenancyMap map[string]string) *corev1.Pod {

	pod.Spec.NodeSelector = tenancyMap

	return pod
}
