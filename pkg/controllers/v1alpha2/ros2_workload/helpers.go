package ros2_workload

import (
	"context"

	robotv1alpha2 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha2"
	"k8s.io/apimachinery/pkg/types"
	"k8s.io/client-go/util/retry"
)

func (r *ROS2WorkloadReconciler) reconcileGetInstance(ctx context.Context, meta types.NamespacedName) (*robotv1alpha2.ROS2Workload, error) {
	instance := &robotv1alpha2.ROS2Workload{}
	err := r.Get(ctx, meta, instance)
	if err != nil {
		return &robotv1alpha2.ROS2Workload{}, err
	}

	return instance, nil
}

func (r *ROS2WorkloadReconciler) reconcileUpdateInstanceStatus(ctx context.Context, instance *robotv1alpha2.ROS2Workload) error {
	return retry.RetryOnConflict(retry.DefaultRetry, func() error {
		instanceLV := &robotv1alpha2.ROS2Workload{}
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
