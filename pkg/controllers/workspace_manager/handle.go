package workspace_manager

import (
	"context"

	robotErr "github.com/robolaunch/robot-operator/internal/error"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
)

func (r *WorkspaceManagerReconciler) reconcileHandleClonerJob(ctx context.Context, instance *robotv1alpha1.WorkspaceManager) error {

	if !instance.Status.ClonerJobStatus.Created {
		instance.Status.Phase = robotv1alpha1.WorkspaceManagerPhaseConfiguringWorkspaces
		err := r.createClonerJob(ctx, instance, instance.GetClonerJobMetadata())
		if err != nil {
			return err
		}
		instance.Status.ClonerJobStatus.Created = true

		return &robotErr.CreatingResourceError{
			ResourceKind:      "Job",
			ResourceName:      instance.GetClonerJobMetadata().Name,
			ResourceNamespace: instance.GetClonerJobMetadata().Namespace,
		}
	}

	if instance.Status.ClonerJobStatus.Phase != string(robotv1alpha1.JobSucceeded) {

		switch instance.Status.ClonerJobStatus.Phase {
		case string(robotv1alpha1.JobActive):
			instance.Status.Phase = robotv1alpha1.WorkspaceManagerPhaseConfiguringWorkspaces
		case string(robotv1alpha1.JobFailed):
			instance.Status.Phase = robotv1alpha1.WorkspaceManagerPhaseFailed
		}

		return &robotErr.WaitingForResourceError{
			ResourceKind:      "Job",
			ResourceName:      instance.GetClonerJobMetadata().Name,
			ResourceNamespace: instance.GetClonerJobMetadata().Namespace,
		}

	}

	return nil
}
