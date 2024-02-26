package build_manager

import (
	"context"

	robotErr "github.com/robolaunch/robot-operator/internal/error"
	"github.com/robolaunch/robot-operator/internal/hybrid"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
)

func (r *BuildManagerReconciler) reconcileHandleConfigMap(ctx context.Context, instance *robotv1alpha1.BuildManager) error {

	if !instance.Status.ScriptConfigMapStatus.Created {
		instance.Status.Phase = robotv1alpha1.BuildManagerCreatingConfigMap
		err := r.createScriptConfigMap(ctx, instance)
		if err != nil {
			return err
		}
		instance.Status.ScriptConfigMapStatus.Created = true

		return &robotErr.CreatingResourceError{
			ResourceKind:      "ConfigMap",
			ResourceName:      instance.GetConfigMapMetadata().Name,
			ResourceNamespace: instance.GetConfigMapMetadata().Namespace,
		}
	}

	return nil
}

func (r *BuildManagerReconciler) reconcileHandleBuilderJobs(ctx context.Context, instance *robotv1alpha1.BuildManager) error {

	robot, err := r.reconcileGetTargetRobot(ctx, instance)
	if err != nil {
		return err
	}

	if hybrid.HasStepInThisInstance(*instance, *robot) {

		instance.Status.Phase = robotv1alpha1.BuildManagerBuildingRobot

		for k := range instance.Status.Steps {
			if instance.Status.Steps[k].Resource.Created {
				if instance.Status.Steps[k].Resource.Phase == string(robotv1alpha1.JobSucceeded) {
					continue
				} else if instance.Status.Steps[k].Resource.Phase == string(robotv1alpha1.JobActive) {
					return &robotErr.WaitingForResourceError{
						ResourceKind:      "Job",
						ResourceName:      instance.Status.Steps[k].Resource.Reference.Name,
						ResourceNamespace: instance.Status.Steps[k].Resource.Reference.Namespace,
					}
				} else if instance.Status.Steps[k].Resource.Phase == string(robotv1alpha1.JobFailed) {
					return &robotErr.WaitingForResourceError{
						ResourceKind:      "Job",
						ResourceName:      instance.Status.Steps[k].Resource.Reference.Name,
						ResourceNamespace: instance.Status.Steps[k].Resource.Reference.Namespace,
					}
				} else {
					return &robotErr.WaitingForResourceError{
						ResourceKind:      "Job",
						ResourceName:      instance.Status.Steps[k].Resource.Reference.Name,
						ResourceNamespace: instance.Status.Steps[k].Resource.Reference.Namespace,
					}
				}

			} else {

				err := r.createBuilderJob(ctx, instance, k)
				if err != nil {
					return err
				}

				stepStatus := instance.Status.Steps[k]
				stepStatus.Resource.Created = true
				instance.Status.Steps[k] = stepStatus

				return &robotErr.CreatingResourceError{
					ResourceKind:      "Job",
					ResourceNamespace: instance.Namespace,
				}
			}
		}
	}

	return nil
}
