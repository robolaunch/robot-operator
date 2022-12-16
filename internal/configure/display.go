package configure

import (
	robotv1alpha1 "github.com/robolaunch/robot-operator/api/v1alpha1"
	"github.com/robolaunch/robot-operator/internal"
	corev1 "k8s.io/api/core/v1"
)

func InjectPodDisplayConfiguration(pod *corev1.Pod, robotVDI robotv1alpha1.RobotVDI) *corev1.Pod {

	placeDisplayEnvironmentVariables(pod)
	placeDisplayVolume(pod, robotVDI)

	return pod
}

func placeDisplayEnvironmentVariables(pod *corev1.Pod) {

	environmentVariables := []corev1.EnvVar{
		internal.Env("DISPLAY", ":0"),
	}

	for k, container := range pod.Spec.Containers {
		container.Env = append(container.Env, environmentVariables...)
		pod.Spec.Containers[k] = container
	}

}

func placeDisplayVolume(pod *corev1.Pod, robotVDI robotv1alpha1.RobotVDI) {

	volume := GetVolumeX11Unix(&robotVDI)
	pod.Spec.Volumes = append(pod.Spec.Volumes, volume)

	volumeMount := GetVolumeMount(internal.X11_UNIX_PATH, volume)

	for k, container := range pod.Spec.Containers {
		container.VolumeMounts = append(container.VolumeMounts, volumeMount)
		pod.Spec.Containers[k] = container
	}

}
