package configure

import (
	robotv1alpha1 "github.com/robolaunch/robot-operator/api/v1alpha1"
	corev1 "k8s.io/api/core/v1"
)

func InjectPodDiscoveryServerConnection(pod *corev1.Pod, robot robotv1alpha1.Robot) *corev1.Pod {

	placeDiscoveryServerEnvironmentVariables(pod, robot)
	placeDiscoveryServerConfigFile(pod, robot)

	return pod
}

func placeDiscoveryServerEnvironmentVariables(pod *corev1.Pod, robot robotv1alpha1.Robot) {

	conn := robot.Status.DiscoveryServerStatus.Status.ConnectionInfo

	environmentVariables := []corev1.EnvVar{
		{
			Name: "FASTRTPS_DEFAULT_PROFILES_FILE",
			ValueFrom: &corev1.EnvVarSource{
				ConfigMapKeyRef: &corev1.ConfigMapKeySelector{
					LocalObjectReference: corev1.LocalObjectReference{
						Name: conn.ConfigMapName,
					},
					Key: "FASTRTPS_DEFAULT_PROFILES_FILE",
				},
			},
		},
		{
			Name: "ROS_DISCOVERY_SERVER",
			ValueFrom: &corev1.EnvVarSource{
				ConfigMapKeyRef: &corev1.ConfigMapKeySelector{
					LocalObjectReference: corev1.LocalObjectReference{
						Name: conn.ConfigMapName,
					},
					Key: "ROS_DISCOVERY_SERVER",
				},
			},
		},
	}

	for _, container := range pod.Spec.Containers {
		container.Env = append(container.Env, environmentVariables...)
	}

}

func placeDiscoveryServerConfigFile(pod *corev1.Pod, robot robotv1alpha1.Robot) {

	conn := robot.Status.DiscoveryServerStatus.Status.ConnectionInfo

	var mode int32 = 511

	volume := corev1.Volume{
		Name: "discovery-server",
		VolumeSource: corev1.VolumeSource{
			ConfigMap: &corev1.ConfigMapVolumeSource{
				LocalObjectReference: corev1.LocalObjectReference{
					Name: conn.ConfigMapName,
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

	for _, container := range pod.Spec.Containers {
		container.VolumeMounts = append(container.VolumeMounts, volumeMount)
	}

}
