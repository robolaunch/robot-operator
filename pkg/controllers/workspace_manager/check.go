package workspace_manager

import (
	"context"

	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	batchv1 "k8s.io/api/batch/v1"
	"k8s.io/apimachinery/pkg/api/errors"
)

func (r *WorkspaceManagerReconciler) reconcileCheckClonerJob(ctx context.Context, instance *robotv1alpha1.WorkspaceManager) error {

	clonerJobQuery := &batchv1.Job{}
	err := r.Get(ctx, *instance.GetClonerJobMetadata(), clonerJobQuery)
	if err != nil && errors.IsNotFound(err) {
		instance.Status.ClonerJobStatus.Created = false
	} else if err != nil {
		return err
	} else {
		switch 1 {
		case int(clonerJobQuery.Status.Succeeded):
			instance.Status.ClonerJobStatus.Phase = robotv1alpha1.JobSucceeded
		case int(clonerJobQuery.Status.Active):
			instance.Status.ClonerJobStatus.Phase = robotv1alpha1.JobActive
		case int(clonerJobQuery.Status.Failed):
			instance.Status.ClonerJobStatus.Phase = robotv1alpha1.JobFailed
		}
	}

	return nil
}

func (r *WorkspaceManagerReconciler) reconcileCheckCleanupJob(ctx context.Context, instance *robotv1alpha1.WorkspaceManager) error {

	isActive := true
	for isActive {

		cleanupJobQuery := &batchv1.Job{}
		err := r.Get(ctx, *instance.GetCleanupJobMetadata(), cleanupJobQuery)
		if err != nil && errors.IsNotFound(err) {
			instance.Status.CleanupJobStatus.Created = false
		} else if err != nil {
			return err
		} else {
			switch 1 {
			case int(cleanupJobQuery.Status.Succeeded):
				instance.Status.CleanupJobStatus.Phase = robotv1alpha1.JobSucceeded
				isActive = false
			case int(cleanupJobQuery.Status.Active):
				instance.Status.CleanupJobStatus.Phase = robotv1alpha1.JobActive
			case int(cleanupJobQuery.Status.Failed):
				instance.Status.CleanupJobStatus.Phase = robotv1alpha1.JobFailed
				isActive = false
			}
		}

	}

	return nil
}
