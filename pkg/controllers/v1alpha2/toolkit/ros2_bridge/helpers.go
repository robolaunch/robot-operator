package ros2_bridge

import (
	"context"

	robotv1alpha2 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha2"
	"k8s.io/apimachinery/pkg/types"
	"k8s.io/client-go/util/retry"
)

func (r *ROS2BridgeReconciler) reconcileGetInstance(ctx context.Context, meta types.NamespacedName) (*robotv1alpha2.ROS2Bridge, error) {
	instance := &robotv1alpha2.ROS2Bridge{}
	err := r.Get(ctx, meta, instance)
	if err != nil {
		return &robotv1alpha2.ROS2Bridge{}, err
	}

	return instance, nil
}

func (r *ROS2BridgeReconciler) reconcileUpdateInstanceStatus(ctx context.Context, instance *robotv1alpha2.ROS2Bridge) error {
	return retry.RetryOnConflict(retry.DefaultRetry, func() error {
		instanceLV := &robotv1alpha2.ROS2Bridge{}
		err := r.Get(ctx, types.NamespacedName{
			Name:      instance.Name,
			Namespace: instance.Namespace,
		}, instanceLV)

		if err == nil {
			instance.ResourceVersion = instanceLV.ResourceVersion
		}

		err1 := r.Status().Update(ctx, instance)
		return err1
	})
}
