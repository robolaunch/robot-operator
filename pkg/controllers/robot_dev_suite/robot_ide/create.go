package robot_ide

import (
	"context"

	robotv1alpha1 "github.com/robolaunch/robot-operator/api/v1alpha1"
)

func (r *RobotIDEReconciler) reconcileCreateService(ctx context.Context, instance *robotv1alpha1.RobotIDE) error {
	return nil
}

func (r *RobotIDEReconciler) reconcileCreatePod(ctx context.Context, instance *robotv1alpha1.RobotIDE) error {
	return nil
}

func (r *RobotIDEReconciler) reconcileCreateIngress(ctx context.Context, instance *robotv1alpha1.RobotIDE) error {
	return nil
}
