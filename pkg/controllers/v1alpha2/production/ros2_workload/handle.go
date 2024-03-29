package ros2_workload

import (
	"context"

	robotErr "github.com/robolaunch/robot-operator/internal/error"

	robotv1alpha2 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha2"
)

func (r *ROS2WorkloadReconciler) reconcileHandleDiscoveryServer(ctx context.Context, instance *robotv1alpha2.ROS2Workload) error {

	if !instance.Status.DiscoveryServerStatus.Resource.Created {
		instance.Status.Phase = robotv1alpha2.ROS2WorkloadPhaseCreatingDiscoveryServer
		err := r.createDiscoveryServer(ctx, instance)
		if err != nil {
			return err
		}
		instance.Status.DiscoveryServerStatus.Resource.Created = true

		return &robotErr.CreatingResourceError{
			ResourceKind:      "DiscoveryServer",
			ResourceName:      instance.GetDiscoveryServerMetadata().Name,
			ResourceNamespace: instance.GetDiscoveryServerMetadata().Namespace,
		}
	}

	return nil
}

func (r *ROS2WorkloadReconciler) reconcileHandleROS2Bridge(ctx context.Context, instance *robotv1alpha2.ROS2Workload) error {

	if instance.Status.DiscoveryServerStatus.Status.ConfigMapStatus.Created && !instance.Status.ROS2BridgeStatus.Resource.Created {
		instance.Status.Phase = robotv1alpha2.ROS2WorkloadPhaseCreatingROS2Bridge
		err := r.createROS2Bridge(ctx, instance)
		if err != nil {
			return err
		}
		instance.Status.ROS2BridgeStatus.Resource.Created = true

		return &robotErr.CreatingResourceError{
			ResourceKind:      "ROS2Bridge",
			ResourceName:      instance.GetROS2BridgeMetadata().Name,
			ResourceNamespace: instance.GetROS2BridgeMetadata().Namespace,
		}
	}

	return nil
}

func (r *ROS2WorkloadReconciler) reconcileHandlePVCs(ctx context.Context, instance *robotv1alpha2.ROS2Workload) error {

	for key, pvcStatus := range instance.Status.PVCStatuses {
		if !pvcStatus.Resource.Created {

			instance.Status.Phase = robotv1alpha2.ROS2WorkloadPhaseCreatingPVCs
			err := r.createPersistentVolumeClaim(ctx, instance, key)
			if err != nil {
				return err
			}

			pvcStatus.Resource.Created = true
			instance.Status.PVCStatuses[key] = pvcStatus

			return &robotErr.CreatingResourceError{
				ResourceKind:      "PersistentVolumeClaim",
				ResourceName:      instance.GetPersistentVolumeClaimMetadata(key).Name,
				ResourceNamespace: instance.GetPersistentVolumeClaimMetadata(key).Namespace,
			}
		}
	}

	return nil
}

func (r *ROS2WorkloadReconciler) reconcileHandleStatefulSets(ctx context.Context, instance *robotv1alpha2.ROS2Workload) error {

	volumesReady := true
	for _, pvcStatus := range instance.Status.PVCStatuses {
		volumesReady = volumesReady && pvcStatus.Resource.Created
	}

	if instance.Status.DiscoveryServerStatus.Status.ConfigMapStatus.Created && volumesReady {
		for key, ssStatus := range instance.Status.StatefulSetStatuses {
			if !ssStatus.Resource.Created {

				instance.Status.Phase = robotv1alpha2.ROS2WorkloadPhaseCreatingStatefulSets
				err := r.createStatefulSet(ctx, instance, key)
				if err != nil {
					return err
				}

				ssStatus.Resource.Created = true
				instance.Status.StatefulSetStatuses[key] = ssStatus

				return &robotErr.CreatingResourceError{
					ResourceKind:      "StatefulSet",
					ResourceName:      instance.GetStatefulSetMetadata(key).Name,
					ResourceNamespace: instance.GetStatefulSetMetadata(key).Namespace,
				}
			}
		}
	}

	return nil
}
