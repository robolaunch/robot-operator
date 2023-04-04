package robot_dev_suite

import (
	"context"
	"reflect"

	"github.com/robolaunch/robot-operator/internal/reference"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	"k8s.io/apimachinery/pkg/api/errors"
)

func (r *RobotDevSuiteReconciler) reconcileCheckRobotVDI(ctx context.Context, instance *robotv1alpha1.RobotDevSuite) error {

	robotVDIQuery := &robotv1alpha1.RobotVDI{}
	err := r.Get(ctx, *instance.GetRobotVDIMetadata(), robotVDIQuery)
	if err != nil {
		if errors.IsNotFound(err) {
			instance.Status.RobotVDIStatus = robotv1alpha1.OwnedResourceStatus{}
		} else {
			return err
		}
	} else {

		if instance.Spec.VDIEnabled {

			if !reflect.DeepEqual(instance.Spec.RobotVDITemplate, robotVDIQuery.Spec) {
				robotVDIQuery.Spec = instance.Spec.RobotVDITemplate
				err = r.Update(ctx, robotVDIQuery)
				if err != nil {
					return err
				}
			}

			instance.Status.RobotVDIStatus.Created = true
			reference.SetReference(&instance.Status.RobotVDIStatus.Reference, robotVDIQuery.TypeMeta, robotVDIQuery.ObjectMeta)
			instance.Status.RobotVDIStatus.Phase = string(robotVDIQuery.Status.Phase)

		} else {

			err := r.Delete(ctx, robotVDIQuery)
			if err != nil {
				return err
			}

		}

	}

	return nil
}

func (r *RobotDevSuiteReconciler) reconcileCheckRobotIDE(ctx context.Context, instance *robotv1alpha1.RobotDevSuite) error {

	robotIDEQuery := &robotv1alpha1.RobotIDE{}
	err := r.Get(ctx, *instance.GetRobotIDEMetadata(), robotIDEQuery)
	if err != nil {
		if errors.IsNotFound(err) {
			instance.Status.RobotIDEStatus = robotv1alpha1.OwnedResourceStatus{}
		} else {
			return err
		}
	} else {

		if instance.Spec.IDEEnabled {

			if !reflect.DeepEqual(instance.Spec.RobotIDETemplate, robotIDEQuery.Spec) {
				robotIDEQuery.Spec = instance.Spec.RobotIDETemplate
				err = r.Update(ctx, robotIDEQuery)
				if err != nil {
					return err
				}
			}

			instance.Status.RobotIDEStatus.Created = true
			reference.SetReference(&instance.Status.RobotIDEStatus.Reference, robotIDEQuery.TypeMeta, robotIDEQuery.ObjectMeta)
			instance.Status.RobotIDEStatus.Phase = string(robotIDEQuery.Status.Phase)

		} else {

			err := r.Delete(ctx, robotIDEQuery)
			if err != nil {
				return err
			}

		}

	}

	return nil
}
