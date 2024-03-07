package ros2_workload

import (
	"context"

	robotv1alpha2 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha2"
	appsv1 "k8s.io/api/apps/v1"
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

func (r *ROS2WorkloadReconciler) registerStatefulSets(ctx context.Context, instance *robotv1alpha2.ROS2Workload) error {

	ssStatuses := []robotv1alpha2.OwnedStatefulSetStatus{}

	if len(instance.Spec.LaunchContainers) != len(instance.Status.StatefulSetStatuses) {

		if len(instance.Status.StatefulSetStatuses) > len(instance.Spec.LaunchContainers) {
			for key := len(instance.Spec.LaunchContainers); key < len(instance.Status.StatefulSetStatuses); key++ {
				statefulSet := appsv1.StatefulSet{}
				err := r.Get(ctx, *instance.GetStatefulSetMetadata(key), &statefulSet)
				if err != nil {
					return err
				}

				err = r.Delete(ctx, &statefulSet)
				if err != nil {
					return err
				}
			}
		}

		for key := range instance.Spec.LaunchContainers {
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

	return nil
}
