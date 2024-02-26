package build_manager

import (
	"context"

	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	"k8s.io/apimachinery/pkg/api/errors"
	"k8s.io/apimachinery/pkg/runtime/schema"
	"sigs.k8s.io/controller-runtime/pkg/controller/controllerutil"
)

func (r *BuildManagerReconciler) reconcileCheckDeletion(ctx context.Context, instance *robotv1alpha1.BuildManager) error {

	bmFinalizer := "buildmanager.robot.roboscale.io/finalizer"

	if instance.DeletionTimestamp.IsZero() {

		if !controllerutil.ContainsFinalizer(instance, bmFinalizer) {
			controllerutil.AddFinalizer(instance, bmFinalizer)
			if err := r.Update(ctx, instance); err != nil {
				return err
			}
		}

	} else {

		if controllerutil.ContainsFinalizer(instance, bmFinalizer) {

			err := r.reconcileDeleteBuilderJobs(ctx, instance)
			if err != nil {
				return err
			}

			err = r.reconcileDeleteConfigMap(ctx, instance)
			if err != nil {
				return err
			}

			controllerutil.RemoveFinalizer(instance, bmFinalizer)
			if err := r.Update(ctx, instance); err != nil {
				return err
			}
		}

		return errors.NewNotFound(schema.GroupResource{
			Group:    instance.GetObjectKind().GroupVersionKind().Group,
			Resource: instance.GetObjectKind().GroupVersionKind().Kind,
		}, instance.Name)
	}

	return nil
}
