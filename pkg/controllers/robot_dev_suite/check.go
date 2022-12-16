package robot_dev_suite

import (
	"context"

	robotv1alpha1 "github.com/robolaunch/robot-operator/api/roboscale.io/v1alpha1"
	"k8s.io/apimachinery/pkg/api/errors"
)

func (r *RobotDevSuiteReconciler) reconcileCheckRobotVDI(ctx context.Context, instance *robotv1alpha1.RobotDevSuite) error {

	if instance.Spec.VDIEnabled {

		robotVDIQuery := &robotv1alpha1.RobotVDI{}
		err := r.Get(ctx, *instance.GetRobotVDIMetadata(), robotVDIQuery)
		if err != nil {
			if errors.IsNotFound(err) {
				instance.Status.RobotVDIStatus.Created = false
			} else {
				return err
			}
		} else {
			instance.Status.RobotVDIStatus.Created = true
			instance.Status.RobotVDIStatus.Phase = robotVDIQuery.Status.Phase
		}

	}

	return nil
}

func (r *RobotDevSuiteReconciler) reconcileCheckRobotIDE(ctx context.Context, instance *robotv1alpha1.RobotDevSuite) error {

	if instance.Spec.IDEEnabled {

		robotIDEQuery := &robotv1alpha1.RobotIDE{}
		err := r.Get(ctx, *instance.GetRobotIDEMetadata(), robotIDEQuery)
		if err != nil {
			if errors.IsNotFound(err) {
				instance.Status.RobotIDEStatus.Created = false
			} else {
				return err
			}
		} else {
			instance.Status.RobotIDEStatus.Created = true
			instance.Status.RobotIDEStatus.Phase = robotIDEQuery.Status.Phase
		}

	}

	return nil
}
