package configure

import (
	robotv1alpha1 "github.com/robolaunch/robot-operator/api/v1alpha1"
	"github.com/robolaunch/robot-operator/internal"
	corev1 "k8s.io/api/core/v1"
)

func InjectPodDisplayConfiguration(pod *corev1.Pod, robot robotv1alpha1.Robot) *corev1.Pod {

	placeDisplayEnvironmentVariables(pod, robot)
	placeDisplayVolume(pod, robot)

	return pod
}

func placeDisplayEnvironmentVariables(pod *corev1.Pod, robot robotv1alpha1.Robot) {

	environmentVariables := []corev1.EnvVar{
		internal.Env("DISPLAY", ":0"),
	}

	for _, container := range pod.Spec.Containers {
		container.Env = append(container.Env, environmentVariables...)
	}

}

func placeDisplayVolume(pod *corev1.Pod, robot robotv1alpha1.Robot) {

	volume := GetVolumeX11Unix(&robot)
	pod.Spec.Volumes = append(pod.Spec.Volumes, volume)

	volumeMount := GetVolumeMount(internal.X11_UNIX_PATH, volume)

	for _, container := range pod.Spec.Containers {
		container.VolumeMounts = append(container.VolumeMounts, volumeMount)
	}

}
