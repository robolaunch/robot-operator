package robot_dev_suite

import (
	"context"

	robotv1alpha1 "github.com/robolaunch/robot-operator/api/v1alpha1"
	"github.com/robolaunch/robot-operator/internal"
	robotErr "github.com/robolaunch/robot-operator/internal/error"
	"github.com/robolaunch/robot-operator/internal/label"
	"k8s.io/apimachinery/pkg/labels"
	"k8s.io/apimachinery/pkg/selection"
	"k8s.io/apimachinery/pkg/types"
	"k8s.io/client-go/util/retry"
	"sigs.k8s.io/controller-runtime/pkg/client"
)

func (r *RobotDevSuiteReconciler) reconcileGetInstance(ctx context.Context, meta types.NamespacedName) (*robotv1alpha1.RobotDevSuite, error) {
	instance := &robotv1alpha1.RobotDevSuite{}
	err := r.Get(ctx, meta, instance)
	if err != nil {
		return &robotv1alpha1.RobotDevSuite{}, err
	}

	return instance, nil
}

func (r *RobotDevSuiteReconciler) reconcileUpdateInstanceStatus(ctx context.Context, instance *robotv1alpha1.RobotDevSuite) error {
	return retry.RetryOnConflict(retry.DefaultRetry, func() error {
		instanceLV := &robotv1alpha1.RobotDevSuite{}
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

func (r *RobotDevSuiteReconciler) reconcileGetTargetRobot(ctx context.Context, instance *robotv1alpha1.RobotDevSuite) (*robotv1alpha1.Robot, error) {
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

func (r *RobotDevSuiteReconciler) reconcileCheckTargetRobot(ctx context.Context, instance *robotv1alpha1.RobotDevSuite) error {

	if label.GetDevSuiteOwned(instance) == "true" {
		instance.Status.Active = true
	} else {
		robot, err := r.reconcileGetTargetRobot(ctx, instance)
		if err != nil {
			return err
		}

		isActive := false
		for _, rds := range robot.Status.AttachedDevObjects {
			if rds.Reference.Kind == instance.Kind && rds.Reference.Name == instance.Name {
				isActive = true
				break
			}
		}

		instance.Status.Active = isActive
	}

	return nil
}

func (r *RobotDevSuiteReconciler) reconcileCheckOtherAttachedResources(ctx context.Context, instance *robotv1alpha1.RobotDevSuite) error {

	if instance.Status.Active && label.GetDevSuiteOwned(instance) != "true" {
		// Get attached build manager objects for this robot
		requirements := []labels.Requirement{}
		newReq, err := labels.NewRequirement(internal.TARGET_ROBOT, selection.In, []string{label.GetTargetRobot(instance)})
		if err != nil {
			return err
		}
		requirements = append(requirements, *newReq)

		robotSelector := labels.NewSelector().Add(requirements...)

		launchManagerList := robotv1alpha1.LaunchManagerList{}
		err = r.List(ctx, &launchManagerList, &client.ListOptions{Namespace: instance.Namespace, LabelSelector: robotSelector})
		if err != nil {
			return err
		}

		for _, lm := range launchManagerList.Items {

			if lm.Status.Active == true {
				return &robotErr.RobotResourcesHasNotBeenReleasedError{
					ResourceKind:      instance.Kind,
					ResourceName:      instance.Name,
					ResourceNamespace: instance.Namespace,
				}
			}

			if lm.Status.Phase != robotv1alpha1.LaunchManagerPhaseInactive {
				return &robotErr.RobotResourcesHasNotBeenReleasedError{
					ResourceKind:      instance.Kind,
					ResourceName:      instance.Name,
					ResourceNamespace: instance.Namespace,
				}
			}
		}

		buildManagerList := robotv1alpha1.BuildManagerList{}
		err = r.List(ctx, &buildManagerList, &client.ListOptions{Namespace: instance.Namespace, LabelSelector: robotSelector})
		if err != nil {
			return err
		}

		for _, bm := range buildManagerList.Items {

			if bm.Name == instance.Name {
				continue
			}

			if bm.Status.Active == true {
				return &robotErr.RobotResourcesHasNotBeenReleasedError{
					ResourceKind:      instance.Kind,
					ResourceName:      instance.Name,
					ResourceNamespace: instance.Namespace,
				}
			}

			if bm.Status.Phase != robotv1alpha1.BuildManagerInactive {
				return &robotErr.RobotResourcesHasNotBeenReleasedError{
					ResourceKind:      instance.Kind,
					ResourceName:      instance.Name,
					ResourceNamespace: instance.Namespace,
				}
			}
		}
	}

	return nil
}
