package configure

import (
	corev1 "k8s.io/api/core/v1"
)

func (cfg *PodSpecConfigInjector) InjectImagePullPolicy(podSpec *corev1.PodSpec) {
	cfg.placeImagePullPolicy(podSpec)
}

func (cfg *PodSpecConfigInjector) placeImagePullPolicy(podSpec *corev1.PodSpec) {

	for k, container := range podSpec.Containers {
		container.ImagePullPolicy = corev1.PullIfNotPresent
		podSpec.Containers[k] = container
	}

	for k, container := range podSpec.InitContainers {
		container.ImagePullPolicy = corev1.PullIfNotPresent
		podSpec.InitContainers[k] = container
	}

}
