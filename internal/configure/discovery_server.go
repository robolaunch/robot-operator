package configure

import (
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	corev1 "k8s.io/api/core/v1"
)

func InjectPodDiscoveryServerConnection(pod *corev1.Pod, connectionInfo robotv1alpha1.ConnectionInfo) *corev1.Pod {

	placeDiscoveryServerEnvironmentVariables(pod, connectionInfo)
	placeDiscoveryServerConfigFile(pod, connectionInfo)

	return pod
}

func placeDiscoveryServerEnvironmentVariables(pod *corev1.Pod, connectionInfo robotv1alpha1.ConnectionInfo) {

	environmentVariables := []corev1.EnvVar{
		{
			Name: "FASTRTPS_DEFAULT_PROFILES_FILE",
			ValueFrom: &corev1.EnvVarSource{
				ConfigMapKeyRef: &corev1.ConfigMapKeySelector{
					LocalObjectReference: corev1.LocalObjectReference{
						Name: connectionInfo.ConfigMapName,
					},
					Key: "FASTRTPS_DEFAULT_PROFILES_FILE",
				},
			},
		},
	}

	for k, container := range pod.Spec.Containers {
		container.Env = append(container.Env, environmentVariables...)
		pod.Spec.Containers[k] = container
	}

}

func placeDiscoveryServerConfigFile(pod *corev1.Pod, connectionInfo robotv1alpha1.ConnectionInfo) {

	var mode int32 = 511

	volume := corev1.Volume{
		Name: "discovery-server",
		VolumeSource: corev1.VolumeSource{
			ConfigMap: &corev1.ConfigMapVolumeSource{
				LocalObjectReference: corev1.LocalObjectReference{
					Name: connectionInfo.ConfigMapName,
				},
				Items: []corev1.KeyToPath{
					{
						Key:  "DISCOVERY_SERVER_CONFIG",
						Path: "super_client_configuration_file.xml",
						Mode: &mode,
					},
				},
			},
		},
	}

	pod.Spec.Volumes = append(pod.Spec.Volumes, volume)

	volumeMount := corev1.VolumeMount{
		Name:      "discovery-server",
		MountPath: "/etc/discovery-server/",
	}

	for k, container := range pod.Spec.Containers {
		container.VolumeMounts = append(container.VolumeMounts, volumeMount)
		pod.Spec.Containers[k] = container
	}

}
