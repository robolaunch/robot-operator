package configure

import (
	corev1 "k8s.io/api/core/v1"
)

func (cfg *PodSpecConfigInjector) InjectLinuxUserAndGroup(podSpec *corev1.PodSpec) {

	var user int64 = 1000
	var group int64 = 3000

	for key, cont := range podSpec.Containers {
		cont.SecurityContext = &corev1.SecurityContext{
			RunAsUser:  &user,
			RunAsGroup: &group,
			Privileged: cont.SecurityContext.Privileged,
		}
		podSpec.Containers[key] = cont
	}
}
