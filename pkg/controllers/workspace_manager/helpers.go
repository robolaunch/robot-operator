package workspace_manager

import (
	"context"

	"github.com/robolaunch/robot-operator/internal/label"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	"k8s.io/apimachinery/pkg/types"
	"k8s.io/client-go/util/retry"
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
