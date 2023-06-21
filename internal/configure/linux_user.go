package configure

import (
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	corev1 "k8s.io/api/core/v1"
)

func InjectLinuxUserAndGroup(pod *corev1.Pod, robot robotv1alpha1.Robot) *corev1.Pod {

	var user int64 = 1000
	var group int64 = 3000

	for key, cont := range pod.Spec.Containers {
		cont.SecurityContext.RunAsUser = &user
		cont.SecurityContext.RunAsGroup = &group
		pod.Spec.Containers[key] = cont
	}

	return pod
}

func InjectLinuxUserAndGroupForPodSpec(podSpec *corev1.PodSpec, robot robotv1alpha1.Robot) *corev1.PodSpec {

	var user int64 = 1000
	var group int64 = 3000

	for key, cont := range podSpec.Containers {
		cont.SecurityContext.RunAsUser = &user
		cont.SecurityContext.RunAsGroup = &group
		podSpec.Containers[key] = cont
	}

	return podSpec
}
