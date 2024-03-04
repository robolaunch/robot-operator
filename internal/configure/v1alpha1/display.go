package configure

import (
	"github.com/robolaunch/robot-operator/internal"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	corev1 "k8s.io/api/core/v1"
)

func (cfg *PodConfigInjector) InjectDisplayConfiguration(pod *corev1.Pod, robotVDI robotv1alpha1.RobotVDI) *corev1.Pod {

	cfg.configurePod(pod, robotVDI)
	for k, container := range pod.Spec.Containers {
		cfg.configureContainer(&container, robotVDI)
		pod.Spec.Containers[k] = container
	}

	return pod
}

func (cfg *PodConfigInjector) InjectDisplayConfigurationForLaunch(pod *corev1.Pod, launchManager robotv1alpha1.LaunchManager, robotVDI robotv1alpha1.RobotVDI) *corev1.Pod {

	cfg.configurePod(pod, robotVDI)
	for launchName, l := range launchManager.Spec.Launches {
		if l.Container.Display {
			for k, container := range pod.Spec.Containers {
				if container.Name == launchName {
					cfg.configureContainer(&pod.Spec.Containers[k], robotVDI)
				}
			}
		}
	}

	return pod
}

func (cfg *PodConfigInjector) configurePod(pod *corev1.Pod, robotVDI robotv1alpha1.RobotVDI) {
	volume := GetVolumeX11Unix(&robotVDI)
	pod.Spec.Volumes = append(pod.Spec.Volumes, volume)
}

func (cfg *PodConfigInjector) configureContainer(container *corev1.Container, robotVDI robotv1alpha1.RobotVDI) {
	volume := GetVolumeX11Unix(&robotVDI)
	volumeMount := GetExternalVolumeMount(internal.X11_UNIX_PATH, volume)
	environmentVariables := []corev1.EnvVar{
		internal.Env("DISPLAY", ":0"),
	}

	container.VolumeMounts = append(container.VolumeMounts, volumeMount)
	container.Env = append(container.Env, environmentVariables...)
}
