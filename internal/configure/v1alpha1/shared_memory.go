package configure

import (
	"github.com/robolaunch/robot-operator/internal"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	corev1 "k8s.io/api/core/v1"
)

func (cfg *PodConfigInjector) InjectSharedMemoryConfiguration(pod *corev1.Pod, robot robotv1alpha1.Robot) *corev1.Pod {

	shmSize := "4Gi" // default shared memory size
	if val, ok := robot.Spec.AdditionalConfigs[internal.SHARED_MEMORY_SIZE_KEY]; ok {
		shmSize = val.Value
	}

	cfg.configureShmVolume(pod, shmSize)
	for k, container := range pod.Spec.Containers {
		cfg.configureShmVolumeMount(&container)
		pod.Spec.Containers[k] = container
	}

	return pod
}

func (cfg *PodConfigInjector) configureShmVolume(pod *corev1.Pod, size string) {
	volume := GetVolumeForSharedMemory(size)
	pod.Spec.Volumes = append(pod.Spec.Volumes, volume)
}

func (cfg *PodConfigInjector) configureShmVolumeMount(container *corev1.Container) {
	volumeMount := GetVolumeMountForSharedMemory()
	container.VolumeMounts = append(container.VolumeMounts, volumeMount)
}
