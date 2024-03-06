package configure

import (
	robotv1alpha2 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha2"
	corev1 "k8s.io/api/core/v1"
)

func (cfg *PodSpecConfigInjector) InjectVolumeConfiguration(podSpec *corev1.PodSpec, pvcStatuses []robotv1alpha2.OwnedPVCStatus) {
	cfg.injectOwnedPVCVolumes(podSpec, pvcStatuses)
}

func (cfg *PodSpecConfigInjector) InjectExternalVolumeConfiguration(podSpec *corev1.PodSpec, evStatuses []robotv1alpha2.ExternalVolumeStatus) {
	cfg.injectExternalPVCVolumes(podSpec, evStatuses)
}

func (cfg *PodSpecConfigInjector) injectOwnedPVCVolumes(podSpec *corev1.PodSpec, pvcStatuses []robotv1alpha2.OwnedPVCStatus) {
	for _, pvcStatus := range pvcStatuses {
		podSpec.Volumes = append(podSpec.Volumes, corev1.Volume{
			Name: pvcStatus.Resource.Reference.Name,
			VolumeSource: corev1.VolumeSource{
				PersistentVolumeClaim: &corev1.PersistentVolumeClaimVolumeSource{
					ClaimName: pvcStatus.Resource.Reference.Name,
				},
			},
		})
	}
}

func (cfg *PodSpecConfigInjector) injectExternalPVCVolumes(podSpec *corev1.PodSpec, evStatuses []robotv1alpha2.ExternalVolumeStatus) {
	for _, evStatus := range evStatuses {
		podSpec.Volumes = append(podSpec.Volumes, corev1.Volume{
			Name: evStatus.Name,
			VolumeSource: corev1.VolumeSource{
				PersistentVolumeClaim: &corev1.PersistentVolumeClaimVolumeSource{
					ClaimName: evStatus.Name,
				},
			},
		})
	}
}
