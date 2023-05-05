package build_manager

import (
	"context"

	"github.com/robolaunch/robot-operator/internal/hybrid"
	"github.com/robolaunch/robot-operator/internal/label"
	"github.com/robolaunch/robot-operator/internal/reference"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
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
			instance.Status.ScriptConfigMapStatus = robotv1alpha1.OwnedResourceStatus{}
		} else {
			return err
		}
	} else {
		instance.Status.ScriptConfigMapStatus.Created = true
		reference.SetReference(&instance.Status.ScriptConfigMapStatus.Reference, configMapQuery.TypeMeta, configMapQuery.ObjectMeta)
	}

	return nil
}

func (r *BuildManagerReconciler) reconcileCheckBuilderJobs(ctx context.Context, instance *robotv1alpha1.BuildManager) error {

	stepStatuses := []robotv1alpha1.StepStatus{}

	robot, err := r.reconcileGetTargetRobot(ctx, instance)
	if err != nil {
		return err
	}

	clusterName := label.GetClusterName(robot)

	for _, step := range instance.Spec.Steps {

		if hybrid.ContainsInstance(step.Instances, clusterName) {
			jobMetadata := types.NamespacedName{
				Namespace: instance.Namespace,
				Name:      instance.Name + "-" + step.Name,
			}

			jobQuery := &batchv1.Job{}
			err := r.Get(ctx, jobMetadata, jobQuery)
			if err != nil {
				if errors.IsNotFound(err) {
					stepStatus := robotv1alpha1.StepStatus{
						Step: step,
						Resource: robotv1alpha1.OwnedResourceStatus{
							Created: false,
						},
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
					Step: step,
					Resource: robotv1alpha1.OwnedResourceStatus{
						Created: true,
						Phase:   string(jobPhase),
					},
				}
				reference.SetReference(&stepStatus.Resource.Reference, jobQuery.TypeMeta, jobQuery.ObjectMeta)

				stepStatuses = append(stepStatuses, stepStatus)
			}
		}
	}

	instance.Status.Steps = stepStatuses

	return nil
}
