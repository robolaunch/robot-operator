package configure

import (
	corev1 "k8s.io/api/core/v1"
)

func (cfg *PodConfigInjector) InjectSharedMemoryConfiguration(pod *corev1.Pod) *corev1.Pod {

	cfg.configureShmVolume(pod)
	for k, container := range pod.Spec.Containers {
		cfg.configureShmVolumeMount(&container)
		pod.Spec.Containers[k] = container
	}

	return pod
}

func (cfg *PodConfigInjector) configureShmVolume(pod *corev1.Pod) {
	volume := GetVolumeForSharedMemory("4Gi")
	pod.Spec.Volumes = append(pod.Spec.Volumes, volume)
}

func (cfg *PodConfigInjector) configureShmVolumeMount(container *corev1.Container) {
	volumeMount := GetVolumeMountForSharedMemory()
	container.VolumeMounts = append(container.VolumeMounts, volumeMount)
}
