package robot_dev_suite

import (
	"context"

	"github.com/robolaunch/robot-operator/internal/resources"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	"k8s.io/apimachinery/pkg/api/errors"
	ctrl "sigs.k8s.io/controller-runtime"
)

func (r *RobotDevSuiteReconciler) reconcileCreateRobotVDI(ctx context.Context, instance *robotv1alpha1.RobotDevSuite) error {

	robotVDI := resources.GetRobotVDI(instance, instance.GetRobotVDIMetadata())

	err := ctrl.SetControllerReference(instance, robotVDI, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, robotVDI)
	if err != nil && errors.IsAlreadyExists(err) {
		return nil
	} else if err != nil {
		return err
	}

	logger.Info("STATUS: Robot VDI is created.")

	return nil
}

func (r *RobotDevSuiteReconciler) reconcileCreateRobotIDE(ctx context.Context, instance *robotv1alpha1.RobotDevSuite) error {

	robotIDE := resources.GetRobotIDE(instance, instance.GetRobotIDEMetadata())

	err := ctrl.SetControllerReference(instance, robotIDE, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, robotIDE)
	if err != nil && errors.IsAlreadyExists(err) {
		return nil
	} else if err != nil {
		return err
	}

	logger.Info("STATUS: Robot IDE is created.")

	return nil
}
