package ros2_workload

import (
	"context"

	robotErr "github.com/robolaunch/robot-operator/internal/error"

	robotv1alpha2 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha2"
)

func (r *ROS2WorkloadReconciler) reconcileHandleDiscoveryServer(ctx context.Context, instance *robotv1alpha2.ROS2Workload) error {

	if !instance.Status.DiscoveryServerStatus.Resource.Created {
		instance.Status.Phase = robotv1alpha2.ROS2WorkloadPhaseCreatingDiscoveryServer
		err := r.createDiscoveryServer(ctx, instance, instance.GetDiscoveryServerMetadata())
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
