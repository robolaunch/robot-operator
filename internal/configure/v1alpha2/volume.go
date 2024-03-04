package configure

import (
	robotv1alpha2 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha2"
	corev1 "k8s.io/api/core/v1"
)

func (cfg *PodConfigInjector) InjectVolumeConfiguration(pod *corev1.Pod, ros2Workload robotv1alpha2.ROS2Workload) {
	cfg.injectOwnedPVCVolumes(pod, ros2Workload)
}

func (cfg *PodConfigInjector) injectOwnedPVCVolumes(pod *corev1.Pod, ros2Workload robotv1alpha2.ROS2Workload) {
	for _, pvcStatus := range ros2Workload.Status.PVCStatuses {
		pod.Spec.Volumes = append(pod.Spec.Volumes, corev1.Volume{
			Name: pvcStatus.Resource.Reference.Name,
			VolumeSource: corev1.VolumeSource{
				PersistentVolumeClaim: &corev1.PersistentVolumeClaimVolumeSource{
					ClaimName: pvcStatus.Resource.Reference.Name,
				},
			},
		})
	}
}
