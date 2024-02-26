package build_manager

import (
	"context"
	"time"

	"github.com/robolaunch/robot-operator/internal/hybrid"
	"github.com/robolaunch/robot-operator/internal/label"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	batchv1 "k8s.io/api/batch/v1"
	corev1 "k8s.io/api/core/v1"
	"k8s.io/apimachinery/pkg/api/errors"
	v1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/types"
	"sigs.k8s.io/controller-runtime/pkg/client"
)

func (r *BuildManagerReconciler) reconcileDeleteConfigMap(ctx context.Context, instance *robotv1alpha1.BuildManager) error {

	configMapQuery := &corev1.ConfigMap{}
	err := r.Get(ctx, *instance.GetConfigMapMetadata(), configMapQuery)
	if err != nil {
		if errors.IsNotFound(err) {
			instance.Status.ScriptConfigMapStatus = robotv1alpha1.OwnedResourceStatus{}
		} else {
			return err
		}
	} else {
		err := r.Delete(ctx, configMapQuery)
		if err != nil {
			return err
		}
		instance.Status.ScriptConfigMapStatus = robotv1alpha1.OwnedResourceStatus{}
	}

	return nil
}

func (r *BuildManagerReconciler) reconcileDeleteBuilderJobs(ctx context.Context, instance *robotv1alpha1.BuildManager) error {

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

				propagationPolicy := v1.DeletePropagationForeground

				err := r.Delete(ctx, jobQuery, &client.DeleteOptions{
					PropagationPolicy: &propagationPolicy,
				})
				if err != nil {
					return err
				}

				// watch until it's deleted
				deleted := false
				for !deleted {
					jobQuery := &batchv1.Job{}
					err := r.Get(ctx, jobMetadata, jobQuery)
					if err != nil && errors.IsNotFound(err) {
						deleted = true
					}
					time.Sleep(time.Second * 1)
				}

				stepStatus := robotv1alpha1.StepStatus{
					Step: step,
					Resource: robotv1alpha1.OwnedResourceStatus{
						Reference: corev1.ObjectReference{
							Name: jobMetadata.Name,
						},
						Created: false,
						Phase:   "",
					},
				}

				stepStatuses = append(stepStatuses, stepStatus)
			}
		}

	}

	instance.Status.Steps = stepStatuses

	return nil
}
