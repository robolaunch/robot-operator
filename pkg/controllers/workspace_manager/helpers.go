package workspace_manager

import (
	"context"

	"github.com/robolaunch/robot-operator/internal"
	robotErr "github.com/robolaunch/robot-operator/internal/error"
	"github.com/robolaunch/robot-operator/internal/label"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	"k8s.io/apimachinery/pkg/labels"
	"k8s.io/apimachinery/pkg/selection"
	"k8s.io/apimachinery/pkg/types"
	"k8s.io/client-go/util/retry"
	"sigs.k8s.io/controller-runtime/pkg/client"
)

func (r *WorkspaceManagerReconciler) reconcileGetInstance(ctx context.Context, meta types.NamespacedName) (*robotv1alpha1.WorkspaceManager, error) {
	instance := &robotv1alpha1.WorkspaceManager{}
	err := r.Get(ctx, meta, instance)
	if err != nil {
		return &robotv1alpha1.WorkspaceManager{}, err
	}

	return instance, nil
}

func (r *WorkspaceManagerReconciler) reconcileUpdateInstanceStatus(ctx context.Context, instance *robotv1alpha1.WorkspaceManager) error {
	return retry.RetryOnConflict(retry.DefaultRetry, func() error {
		instanceLV := &robotv1alpha1.WorkspaceManager{}
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

func (r *WorkspaceManagerReconciler) reconcileGetTargetRobot(ctx context.Context, instance *robotv1alpha1.WorkspaceManager) (*robotv1alpha1.Robot, error) {
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

func (r *WorkspaceManagerReconciler) reconcileCleanup(ctx context.Context, instance *robotv1alpha1.WorkspaceManager) error {

	err := r.reconcileDeleteClonerJob(ctx, instance)
	if err != nil {
		return err
	}

	err = r.reconcileDeleteCleanupJob(ctx, instance)
	if err != nil {
		return err
	}

	err = r.createCleanupJob(ctx, instance, instance.GetCleanupJobMetadata())
	if err != nil {
		return err
	}

	err = r.reconcileCheckCleanupJob(ctx, instance)
	if err != nil {
		return err
	}

	err = r.reconcileDeleteCleanupJob(ctx, instance)
	if err != nil {
		return err
	}
	return nil
}

func (r *WorkspaceManagerReconciler) reconcileCheckOtherAttachedResources(ctx context.Context, instance *robotv1alpha1.WorkspaceManager) error {

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

	robotDevSuiteList := robotv1alpha1.RobotDevSuiteList{}
	err = r.List(ctx, &robotDevSuiteList, &client.ListOptions{Namespace: instance.Namespace, LabelSelector: robotSelector.Add()})
	if err != nil {
		return err
	}

	for _, rds := range robotDevSuiteList.Items {

		if rds.Status.Active {
			return &robotErr.RobotResourcesHasNotBeenReleasedError{
				ResourceKind:      instance.Kind,
				ResourceName:      instance.Name,
				ResourceNamespace: instance.Namespace,
			}
		}

		if rds.Status.Phase != robotv1alpha1.RobotDevSuitePhaseInactive {
			return &robotErr.RobotResourcesHasNotBeenReleasedError{
				ResourceKind:      instance.Kind,
				ResourceName:      instance.Name,
				ResourceNamespace: instance.Namespace,
			}
		}
	}

	launchManagerList := robotv1alpha1.LaunchManagerList{}
	err = r.List(ctx, &launchManagerList, &client.ListOptions{Namespace: instance.Namespace, LabelSelector: robotSelector})
	if err != nil {
		return err
	}

	for _, lm := range launchManagerList.Items {

		if lm.Status.Active {
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

		if bm.Status.Active {
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

	return nil
}

func (r *WorkspaceManagerReconciler) reconcileCheckUpdates(ctx context.Context, instance *robotv1alpha1.WorkspaceManager) error {

	switch instance.Spec.UpdateNeeded {
	case true:

		instance.Spec.UpdateNeeded = false
		err := r.Update(ctx, instance, &client.UpdateOptions{})
		if err != nil {
			return err
		}

		instance.Status.Version++
		instance.Status.Phase = robotv1alpha1.WorkspaceManagerPhaseConfiguringWorkspaces
		instance.Status.CleanupJobStatus = robotv1alpha1.OwnedResourceStatus{}
		instance.Status.ClonerJobStatus = robotv1alpha1.OwnedResourceStatus{}

		err = r.reconcileUpdateInstanceStatus(ctx, instance)
		if err != nil {
			return err
		}

		err = r.reconcileCleanup(ctx, instance)
		if err != nil {
			return err
		}

	}

	return nil
}
