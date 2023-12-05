package configure

import (
	corev1 "k8s.io/api/core/v1"
)

func (cfg *PodConfigInjector) InjectBackgroundConfigFiles(pod *corev1.Pod, configMap corev1.ConfigMap) *corev1.Pod {

	cfg.placeBackgroundConfigFiles(pod, configMap)

	return pod
}

func (cfg *PodConfigInjector) placeBackgroundConfigFiles(pod *corev1.Pod, configMap corev1.ConfigMap) {

	var mode int32 = 511

	volume := corev1.Volume{
		Name: "background-config",
		VolumeSource: corev1.VolumeSource{
			ConfigMap: &corev1.ConfigMapVolumeSource{
				LocalObjectReference: corev1.LocalObjectReference{
					Name: configMap.Name,
				},
			},
		},
	}

	for key := range configMap.Data {
		volume.VolumeSource.ConfigMap.Items = append(volume.VolumeSource.ConfigMap.Items, corev1.KeyToPath{
			Key:  key,
			Path: key,
			Mode: &mode,
		})
	}

	pod.Spec.Volumes = append(pod.Spec.Volumes, volume)

	volumeMount := corev1.VolumeMount{
		Name:      "background-config",
		MountPath: "/etc/robolaunch/services/config/",
	}

	for k, container := range pod.Spec.Containers {
		container.VolumeMounts = append(container.VolumeMounts, volumeMount)
		pod.Spec.Containers[k] = container
	}

}
