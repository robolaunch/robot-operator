package configure

import (
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	batchv1 "k8s.io/api/batch/v1"
	corev1 "k8s.io/api/core/v1"
)

func (cfg *ContainerConfigInjector) InjectVolumeMountConfiguration(container *corev1.Container, robot robotv1alpha1.Robot, mountPrefix string) *corev1.Container {

	container.VolumeMounts = append(container.VolumeMounts, getVolumeMountsForPersistentDirs(robot, mountPrefix)...)

	return container
}

func (cfg *PodConfigInjector) InjectVolumeConfiguration(pod *corev1.Pod, robot robotv1alpha1.Robot) *corev1.Pod {

	pod.Spec.Volumes = append(pod.Spec.Volumes, getVolumesForPersistentDirs(robot)...)

	return pod
}

func (cfg *JobConfigInjector) InjectVolumeConfiguration(job *batchv1.Job, robot robotv1alpha1.Robot) *batchv1.Job {

	podSpec := job.Spec.Template.Spec

	podSpec.Volumes = append(podSpec.Volumes, getVolumesForPersistentDirs(robot)...)

	job.Spec.Template.Spec = podSpec

	return job
}

func getVolumesForPersistentDirs(robot robotv1alpha1.Robot) []corev1.Volume {

	volumes := []corev1.Volume{}
	for _, pDir := range robot.Status.PersistentDirectories {
		volumes = append(volumes, getVolumeForPersistentDir(pDir))
	}

	return volumes
}

func getVolumeMountsForPersistentDirs(robot robotv1alpha1.Robot, mountPrefix string) []corev1.VolumeMount {

	volumeMounts := []corev1.VolumeMount{}
	for _, pDir := range robot.Status.PersistentDirectories {
		volumeMounts = append(volumeMounts, getVolumeMountForPersistentDir(mountPrefix, pDir))
	}

	return volumeMounts
}

func getVolumeForPersistentDir(pDir robotv1alpha1.PersistentDirectory) corev1.Volume {
	volume := corev1.Volume{
		Name: pDir.Status.Reference.Name,
		VolumeSource: corev1.VolumeSource{
			PersistentVolumeClaim: &corev1.PersistentVolumeClaimVolumeSource{
				ClaimName: pDir.Status.Reference.Name,
			},
		},
	}

	return volume
}

func getVolumeMountForPersistentDir(
	mountPrefix string,
	pDir robotv1alpha1.PersistentDirectory,
) corev1.VolumeMount {

	volumeMount := corev1.VolumeMount{
		Name:      pDir.Status.Reference.Name,
		MountPath: mountPrefix + pDir.Path,
	}

	return volumeMount
}

// *Deprecated*
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

// *Deprecated*
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

// *Deprecated*
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

// *Deprecated*
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

func GetVolumeX11Unix(robotVDI *robotv1alpha1.RobotVDI) corev1.Volume {

	volume := corev1.Volume{
		Name: "x11-unix",
		VolumeSource: corev1.VolumeSource{
			PersistentVolumeClaim: &corev1.PersistentVolumeClaimVolumeSource{
				ClaimName: robotVDI.GetRobotVDIPVCMetadata().Name,
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

// *Deprecated*
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
