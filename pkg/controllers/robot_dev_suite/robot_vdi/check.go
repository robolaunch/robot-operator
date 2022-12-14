package robot_vdi

import (
	"context"

	robotv1alpha1 "github.com/robolaunch/robot-operator/api/v1alpha1"
)

func (r *RobotVDIReconciler) reconcileCheckPVC(ctx context.Context, instance *robotv1alpha1.RobotVDI) error {

	if !instance.Spec.MainDisplay {

	}

	return nil
}

func (r *RobotVDIReconciler) reconcileCheckServices(ctx context.Context, instance *robotv1alpha1.RobotVDI) error {
	return nil
}

func (r *RobotVDIReconciler) reconcileCheckPod(ctx context.Context, instance *robotv1alpha1.RobotVDI) error {
	return nil
}

func (r *RobotVDIReconciler) reconcileCheckIngress(ctx context.Context, instance *robotv1alpha1.RobotVDI) error {
	return nil
}
