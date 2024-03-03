package ros2_workload

import (
	"context"

	robotv1alpha2 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha2"
	corev1 "k8s.io/api/core/v1"
)

func (r *ROS2WorkloadReconciler) registerPVCs(ctx context.Context, instance *robotv1alpha2.ROS2Workload) error {

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

	return nil
}
