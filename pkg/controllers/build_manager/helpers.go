package build_manager

import (
	"context"
	"time"

	"github.com/robolaunch/robot-operator/internal"
	"github.com/robolaunch/robot-operator/internal/handle"
	"github.com/robolaunch/robot-operator/internal/label"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	"k8s.io/apimachinery/pkg/labels"
	"k8s.io/apimachinery/pkg/selection"
	"k8s.io/apimachinery/pkg/types"
	"k8s.io/client-go/util/retry"
	"sigs.k8s.io/controller-runtime/pkg/reconcile"
)

func (r *BuildManagerReconciler) reconcileGetInstance(ctx context.Context, meta types.NamespacedName) (*robotv1alpha1.BuildManager, error) {
	instance := &robotv1alpha1.BuildManager{}
	err := r.Get(ctx, meta, instance)
	if err != nil {
		return &robotv1alpha1.BuildManager{}, err
	}

	return instance, nil
}

func (r *BuildManagerReconciler) reconcileUpdateInstanceStatus(ctx context.Context, instance *robotv1alpha1.BuildManager) error {
	return retry.RetryOnConflict(retry.DefaultRetry, func() error {
		instanceLV := &robotv1alpha1.BuildManager{}
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

func (r *BuildManagerReconciler) reconcileGetTargetRobot(ctx context.Context, instance *robotv1alpha1.BuildManager) (*robotv1alpha1.Robot, error) {
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

func (r *BuildManagerReconciler) reconcileCheckTargetRobot(ctx context.Context, instance *robotv1alpha1.BuildManager) error {
	robot, err := r.reconcileGetTargetRobot(ctx, instance)
	if err != nil {
		return err
	}

	if robot.Status.AttachedBuildObject.Reference.Kind == instance.Kind && robot.Status.AttachedBuildObject.Reference.Name == instance.Name {
		instance.Status.Active = true
	} else {
		instance.Status.Active = false
	}

	return nil
}

func (r *BuildManagerReconciler) reconcileCheckOtherAttachedResources(ctx context.Context, instance *robotv1alpha1.BuildManager) error {

	if instance.Status.Active {
		// Get attached build manager objects for this robot
		requirements := []labels.Requirement{}
		targetReq, err := labels.NewRequirement(internal.TARGET_ROBOT_LABEL_KEY, selection.In, []string{label.GetTargetRobot(instance)})
		if err != nil {
			return err
		}

		ownedReq, err := labels.NewRequirement(internal.ROBOT_DEV_SUITE_OWNED, selection.DoesNotExist, []string{})
		if err != nil {
			return err
		}
		requirements = append(requirements, *targetReq, *ownedReq)

		robotSelector := labels.NewSelector().Add(requirements...)

		err = handle.CheckIfAnyRDSActive(ctx, r.Client, instance, robotSelector)
		if err != nil {
			return err
		}

		err = handle.CheckIfAnyLMActive(ctx, r.Client, instance, robotSelector)
		if err != nil {
			return err
		}

		err = handle.CheckIfAnyBMActive(ctx, r.Client, instance, robotSelector)
		if err != nil {
			return err
		}

	}

	return nil
}

func Requeue(result *reconcile.Result) {
	result.Requeue = true
	result.RequeueAfter = 3 * time.Second
}
