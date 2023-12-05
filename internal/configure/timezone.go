package configure

import (
	"github.com/robolaunch/robot-operator/internal/label"
	corev1 "k8s.io/api/core/v1"
)

func (cfg *PodConfigInjector) InjectTimezone(pod *corev1.Pod, node corev1.Node) *corev1.Pod {

	cfg.placeTimezone(pod, node)

	return pod
}

func (cfg *PodConfigInjector) placeTimezone(pod *corev1.Pod, node corev1.Node) {

	timezone := label.GetTimezone(&node)
	if timezone.Continent == "" || timezone.City == "" {
		return
	}

	volume := corev1.Volume{
		Name: "tz-config",
		VolumeSource: corev1.VolumeSource{
			HostPath: &corev1.HostPathVolumeSource{
				Path: "/usr/share/zoneinfo/" + timezone.Continent + "/" + timezone.City,
			},
		},
	}

	pod.Spec.Volumes = append(pod.Spec.Volumes, volume)

	volumeMount := corev1.VolumeMount{
		Name:      "tz-config",
		MountPath: "/etc/localtime",
	}

	for k, container := range pod.Spec.Containers {
		container.VolumeMounts = append(container.VolumeMounts, volumeMount)
		pod.Spec.Containers[k] = container
	}

}
