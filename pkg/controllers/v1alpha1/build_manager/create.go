package build_manager

import (
	"context"

	resources "github.com/robolaunch/robot-operator/internal/resources/v1alpha1"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	"k8s.io/apimachinery/pkg/api/errors"
	ctrl "sigs.k8s.io/controller-runtime"
)

func (r *BuildManagerReconciler) createScriptConfigMap(ctx context.Context, instance *robotv1alpha1.BuildManager) error {
	scriptConfigMap, err := resources.GetConfigMap(instance)
	if err != nil {
		return err
	}

	err = ctrl.SetControllerReference(instance, scriptConfigMap, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, scriptConfigMap)
	if err != nil && errors.IsAlreadyExists(err) {
		return nil
	} else if err != nil {
		return err
	}

	logger.Info("STATUS: Script ConfigMap is created.")
	return nil
}

func (r *BuildManagerReconciler) createBuilderJob(ctx context.Context, instance *robotv1alpha1.BuildManager, jobKey int) error {

	robot, err := r.reconcileGetTargetRobot(ctx, instance)
	if err != nil {
		return err
	}

	job := resources.GetBuildJob(
		instance,
		robot,
		instance.Status.Steps[jobKey].Step,
	)

	err = ctrl.SetControllerReference(instance, job, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, job)
	if err != nil && errors.IsAlreadyExists(err) {
		return nil
	} else if err != nil {
		return err
	}

	logger.Info("STATUS: Builder job " + instance.Status.Steps[jobKey].Resource.Reference.Name + " is started.")

	step := instance.Status.Steps[jobKey]
	step.Resource.Created = true
	instance.Status.Steps[jobKey] = step

	return nil
}
