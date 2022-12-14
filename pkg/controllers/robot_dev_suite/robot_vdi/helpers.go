package robot_vdi

import (
	"context"

	robotv1alpha1 "github.com/robolaunch/robot-operator/api/v1alpha1"
	"github.com/robolaunch/robot-operator/internal/label"
	"k8s.io/apimachinery/pkg/types"
	"k8s.io/client-go/util/retry"
)

func (r *RobotVDIReconciler) reconcileGetInstance(ctx context.Context, meta types.NamespacedName) (*robotv1alpha1.RobotVDI, error) {
	instance := &robotv1alpha1.RobotVDI{}
	err := r.Get(ctx, meta, instance)
	if err != nil {
		return &robotv1alpha1.RobotVDI{}, err
	}

	return instance, nil
}

func (r *RobotVDIReconciler) reconcileUpdateInstanceStatus(ctx context.Context, instance *robotv1alpha1.RobotVDI) error {
	return retry.RetryOnConflict(retry.DefaultRetry, func() error {
		instanceLV := &robotv1alpha1.RobotVDI{}
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

func (r *RobotVDIReconciler) reconcileGetTargetRobot(ctx context.Context, instance *robotv1alpha1.RobotVDI) (*robotv1alpha1.Robot, error) {
	robot := &robotv1alpha1.Robot{}
	err := r.Get(ctx, types.NamespacedName{
		Namespace: instance.Namespace,
		Name:      label.GetTargetRobot(instance),
	}, robot)
	if err != nil {
		return nil, err
	}

	return robot, nil
}
