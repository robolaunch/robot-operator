package configure

import (
	robotv1alpha1 "github.com/robolaunch/robot-operator/api/v1alpha1"
	corev1 "k8s.io/api/core/v1"
)

func GetVolumeVar(robot *robotv1alpha1.Robot) corev1.Volume {

	volume := corev1.Volume{
		Name: "var",
		VolumeSource: corev1.VolumeSource{
			PersistentVolumeClaim: &corev1.PersistentVolumeClaimVolumeSource{
				ClaimName: robot.GetPVCVarMetadata().Name,
			},
		},
	}

	return volume
}

func GetVolumeOpt(robot *robotv1alpha1.Robot) corev1.Volume {

	volume := corev1.Volume{
		Name: "opt",
		VolumeSource: corev1.VolumeSource{
			PersistentVolumeClaim: &corev1.PersistentVolumeClaimVolumeSource{
				ClaimName: robot.GetPVCOptMetadata().Name,
			},
		},
	}

	return volume
}

func GetVolumeUsr(robot *robotv1alpha1.Robot) corev1.Volume {

	volume := corev1.Volume{
		Name: "usr",
		VolumeSource: corev1.VolumeSource{
			PersistentVolumeClaim: &corev1.PersistentVolumeClaimVolumeSource{
				ClaimName: robot.GetPVCUsrMetadata().Name,
			},
		},
	}

	return volume
}

func GetVolumeEtc(robot *robotv1alpha1.Robot) corev1.Volume {

	volume := corev1.Volume{
		Name: "etc",
		VolumeSource: corev1.VolumeSource{
			PersistentVolumeClaim: &corev1.PersistentVolumeClaimVolumeSource{
				ClaimName: robot.GetPVCEtcMetadata().Name,
			},
		},
	}

	return volume
}

func GetVolumeX11Unix(robot *robotv1alpha1.Robot) corev1.Volume {

	volume := corev1.Volume{
		Name: "x11-unix",
		VolumeSource: corev1.VolumeSource{
			PersistentVolumeClaim: &corev1.PersistentVolumeClaimVolumeSource{
				ClaimName: robot.GetPVCDisplayMetadata().Name,
			},
		},
	}

	return volume
}

func GetVolumeWorkspace(robot *robotv1alpha1.Robot) corev1.Volume {

	volume := corev1.Volume{
		Name: "workspace",
		VolumeSource: corev1.VolumeSource{
			PersistentVolumeClaim: &corev1.PersistentVolumeClaimVolumeSource{
				ClaimName: robot.GetPVCWorkspaceMetadata().Name,
			},
		},
	}

	return volume
}

func GetVolumeDshm() corev1.Volume {

	volume := corev1.Volume{
		Name: "dshm",
		VolumeSource: corev1.VolumeSource{
			EmptyDir: &corev1.EmptyDirVolumeSource{
				Medium: corev1.StorageMediumMemory,
			},
		},
	}

	return volume
}

func GetVolumeXglCache() corev1.Volume {

	volume := corev1.Volume{
		Name: "xgl-cache-vol",
		VolumeSource: corev1.VolumeSource{
			EmptyDir: &corev1.EmptyDirVolumeSource{},
		},
	}

	return volume
}

func GetVolumeMount(
	mountPrefix string,
	volume corev1.Volume,
) corev1.VolumeMount {
	mountPath := mountPrefix + volume.Name

	if volume.Name == "workspace" {
		mountPath = mountPrefix
	}
	if volume.Name == "config-volume" {
		mountPath = mountPrefix
	}
	if volume.Name == "x11-unix" {
		mountPath = mountPrefix
	}
	if volume.Name == "dshm" {
		mountPath = mountPrefix
	}
	if volume.Name == "xgl-cache-vol" {
		mountPath = mountPrefix
	}

	volumeMount := corev1.VolumeMount{
		Name:      volume.Name,
		MountPath: mountPath,
	}

	return volumeMount
}

func GetVolumeConfigMaps(buildManager *robotv1alpha1.BuildManager) corev1.Volume {

	configKeys := []corev1.KeyToPath{}

	for _, step := range buildManager.Spec.Steps {
		if step.Script != "" {
			var stepMod int32 = 511

			configKeys = append(configKeys,
				corev1.KeyToPath{
					Key:  step.Name,
					Path: "scripts/" + step.Name,
					Mode: &stepMod,
				},
			)
		}
	}

	volume := corev1.Volume{
		Name: "config-volume",
		VolumeSource: corev1.VolumeSource{
			ConfigMap: &corev1.ConfigMapVolumeSource{
				LocalObjectReference: corev1.LocalObjectReference{
					Name: buildManager.GetConfigMapMetadata().Name,
				},
				Items: configKeys,
			},
		},
	}

	return volume
}
