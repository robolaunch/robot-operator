package robot

import (
	"context"

	robotv1alpha1 "github.com/robolaunch/robot-operator/api/v1alpha1"
)

func (r *RobotReconciler) reconcileHandleAttachments(ctx context.Context, instance *robotv1alpha1.Robot) error {

	switch instance.Spec.Development {
	case true:

		// attach development suite
		err := r.reconcileAttachDevObject(ctx, instance)
		if err != nil {
			return err
		}

	case false:

		// select attached build object
		err := r.reconcileAttachBuildObject(ctx, instance)
		if err != nil {
			return err
		}

		switch instance.Status.AttachedBuildObject.Status.Phase {
		case robotv1alpha1.BuildManagerReady:

			// select attached launch object
			err := r.reconcileAttachLaunchObject(ctx, instance)
			if err != nil {
				return err
			}

		}

	}

	return nil
}
