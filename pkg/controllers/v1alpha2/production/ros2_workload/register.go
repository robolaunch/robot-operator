package ros2_workload

import (
	robotv1alpha2 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha2"
	corev1 "k8s.io/api/core/v1"
)

func (r *ROS2WorkloadReconciler) registerPVCs(instance *robotv1alpha2.ROS2Workload) {

	pvcStatuses := []robotv1alpha2.OwnedPVCStatus{}

	if len(instance.Spec.VolumeClaimTemplates) != len(instance.Status.PVCStatuses) {
		for key := range instance.Spec.VolumeClaimTemplates {
			pvcStatus := robotv1alpha2.OwnedPVCStatus{
				Resource: robotv1alpha2.OwnedResourceStatus{
					Reference: corev1.ObjectReference{
						Namespace: instance.Namespace,
						Name:      instance.GetPersistentVolumeClaimMetadata(key).Name,
					},
				},
			}
			pvcStatuses = append(pvcStatuses, pvcStatus)
		}
		instance.Status.PVCStatuses = pvcStatuses
	}
}

func (r *ROS2WorkloadReconciler) registerStatefulSets(instance *robotv1alpha2.ROS2Workload) {

	ssStatuses := []robotv1alpha2.OwnedStatefulSetStatus{}

	if len(instance.Spec.Containers) != len(instance.Status.StatefulSetStatuses) {
		for key := range instance.Spec.Containers {
			ssStatus := robotv1alpha2.OwnedStatefulSetStatus{
				Resource: robotv1alpha2.OwnedResourceStatus{
					Reference: corev1.ObjectReference{
						Namespace: instance.Namespace,
						Name:      instance.GetStatefulSetMetadata(key).Name,
					},
				},
			}
			ssStatuses = append(ssStatuses, ssStatus)
		}
		instance.Status.StatefulSetStatuses = ssStatuses
	}
}
