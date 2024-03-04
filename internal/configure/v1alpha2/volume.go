package configure

import (
	robotv1alpha2 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha2"
	appsv1 "k8s.io/api/apps/v1"
	corev1 "k8s.io/api/core/v1"
)

func (cfg *StatefulSetConfigInjector) InjectVolumeConfiguration(statefulSet *appsv1.StatefulSet, ros2Workload robotv1alpha2.ROS2Workload) {
	cfg.injectOwnedPVCVolumes(statefulSet, ros2Workload)
}

func (cfg *StatefulSetConfigInjector) injectOwnedPVCVolumes(statefulSet *appsv1.StatefulSet, ros2Workload robotv1alpha2.ROS2Workload) {
	for _, pvcStatus := range ros2Workload.Status.PVCStatuses {
		statefulSet.Spec.Template.Spec.Volumes = append(statefulSet.Spec.Template.Spec.Volumes, corev1.Volume{
			Name: pvcStatus.Resource.Reference.Name,
			VolumeSource: corev1.VolumeSource{
				PersistentVolumeClaim: &corev1.PersistentVolumeClaimVolumeSource{
					ClaimName: pvcStatus.Resource.Reference.Name,
				},
			},
		})
	}
}
