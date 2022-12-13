package build_manager

import (
	"context"

	robotv1alpha1 "github.com/robolaunch/robot-operator/api/v1alpha1"
	batchv1 "k8s.io/api/batch/v1"
	corev1 "k8s.io/api/core/v1"
	"k8s.io/apimachinery/pkg/api/errors"
	"k8s.io/apimachinery/pkg/types"
)

func (r *BuildManagerReconciler) reconcileCheckConfigMap(ctx context.Context, instance *robotv1alpha1.BuildManager) error {

	configMapQuery := &corev1.ConfigMap{}
	err := r.Get(ctx, *instance.GetConfigMapMetadata(), configMapQuery)
	if err != nil {
		if errors.IsNotFound(err) {
			instance.Status.ScriptConfigMapStatus.Created = false
		} else {
			return err
		}
	} else {
		instance.Status.ScriptConfigMapStatus.Created = true
	}

	return nil
}

func (r *BuildManagerReconciler) reconcileCheckBuilderJobs(ctx context.Context, instance *robotv1alpha1.BuildManager) error {

	stepStatuses := []robotv1alpha1.StepStatus{}
	for _, step := range instance.Spec.Steps {
		jobMetadata := types.NamespacedName{
			Namespace: instance.Namespace,
			Name:      instance.Name + "-" + step.Name,
		}

		jobQuery := &batchv1.Job{}
		err := r.Get(ctx, jobMetadata, jobQuery)
		if err != nil {
			if errors.IsNotFound(err) {
				stepStatus := robotv1alpha1.StepStatus{
					Step:       step,
					JobName:    jobMetadata.Name,
					JobCreated: false,
				}

				stepStatuses = append(stepStatuses, stepStatus)
			} else {
				return err
			}
		} else {
			var jobPhase robotv1alpha1.JobPhase
			switch 1 {
			case int(jobQuery.Status.Succeeded):
				jobPhase = robotv1alpha1.JobSucceeded
			case int(jobQuery.Status.Active):
				jobPhase = robotv1alpha1.JobActive
			case int(jobQuery.Status.Failed):
				jobPhase = robotv1alpha1.JobFailed
			}

			stepStatus := robotv1alpha1.StepStatus{
				Step:       step,
				JobName:    jobMetadata.Name,
				JobCreated: true,
				JobPhase:   jobPhase,
			}

			stepStatuses = append(stepStatuses, stepStatus)
		}
	}

	instance.Status.Steps = stepStatuses

	return nil
}
