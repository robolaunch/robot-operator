package build_manager

import (
	"context"

	"github.com/robolaunch/robot-operator/internal"
	"github.com/robolaunch/robot-operator/internal/label"
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

	robot, err := r.reconcileGetTargetRobot(ctx, instance)
	if err != nil {
		return err
	}

	for _, step := range instance.Spec.Steps {

		runOnCluster := false
		if physicalInstance, ok := step.Selector[internal.PHYSICAL_INSTANCE_LABEL_KEY]; ok {
			if physicalInstance == label.GetClusterName(robot) {
				runOnCluster = true
			}
		} else if cloudInstance, ok := step.Selector[internal.CLOUD_INSTANCE_LABEL_KEY]; ok {
			if cloudInstance == label.GetClusterName(robot) {
				runOnCluster = true
			}
		} else {
			runOnCluster = true
		}

		if runOnCluster {
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
							Reference: corev1.ObjectReference{
								Name: jobMetadata.Name,
							},
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
						Reference: corev1.ObjectReference{
							Name: jobMetadata.Name,
						},
						Created: true,
						Phase:   string(jobPhase),
					},
				}

				stepStatuses = append(stepStatuses, stepStatus)
			}
		}
	}

	instance.Status.Steps = stepStatuses

	return nil
}
