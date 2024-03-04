package configure

import (
	corev1 "k8s.io/api/core/v1"
)

func (cfg *PodConfigInjector) InjectImagePullPolicy(pod *corev1.Pod) *corev1.Pod {

	cfg.placeImagePullPolicy(pod)

	return pod
}

func (cfg *PodConfigInjector) placeImagePullPolicy(pod *corev1.Pod) {

	for k, container := range pod.Spec.Containers {
		container.ImagePullPolicy = corev1.PullIfNotPresent
		pod.Spec.Containers[k] = container
	}

	for k, container := range pod.Spec.InitContainers {
		container.ImagePullPolicy = corev1.PullIfNotPresent
		pod.Spec.InitContainers[k] = container
	}

}
