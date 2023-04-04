package workspace_manager

import (
	"context"
	"time"

	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	batchv1 "k8s.io/api/batch/v1"
	"k8s.io/apimachinery/pkg/api/errors"
	v1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"sigs.k8s.io/controller-runtime/pkg/client"
)

func (r *WorkspaceManagerReconciler) reconcileDeleteClonerJob(ctx context.Context, instance *robotv1alpha1.WorkspaceManager) error {

	clonerJobQuery := &batchv1.Job{}
	err := r.Get(ctx, *instance.GetClonerJobMetadata(), clonerJobQuery)
	if err != nil {
		if errors.IsNotFound(err) {
			instance.Status.ClonerJobStatus = robotv1alpha1.OwnedJobStatus{}
		} else {
			return err
		}
	} else {

		propagationPolicyFG := v1.DeletePropagationForeground

		err := r.Delete(ctx, clonerJobQuery, &client.DeleteOptions{
			PropagationPolicy: &propagationPolicyFG,
		})
		if err != nil {
			return err
		}

		// watch until it's deleted
		deleted := false
		for !deleted {
			jobQuery := &batchv1.Job{}
			err := r.Get(ctx, *instance.GetClonerJobMetadata(), jobQuery)
			if err != nil && errors.IsNotFound(err) {
				deleted = true
			}
			time.Sleep(time.Second * 1)
		}

		instance.Status.ClonerJobStatus = robotv1alpha1.OwnedJobStatus{}
	}

	return nil
}

func (r *WorkspaceManagerReconciler) reconcileDeleteCleanupJob(ctx context.Context, instance *robotv1alpha1.WorkspaceManager) error {

	cleanupJobQuery := &batchv1.Job{}
	err := r.Get(ctx, *instance.GetCleanupJobMetadata(), cleanupJobQuery)
	if err != nil {
		if errors.IsNotFound(err) {
			instance.Status.CleanupJobStatus = robotv1alpha1.OwnedJobStatus{}
		} else {
			return err
		}
	} else {

		propagationPolicyFG := v1.DeletePropagationForeground

		err := r.Delete(ctx, cleanupJobQuery, &client.DeleteOptions{
			PropagationPolicy: &propagationPolicyFG,
		})
		if err != nil {
			return err
		}

		// watch until it's deleted
		deleted := false
		for !deleted {
			jobQuery := &batchv1.Job{}
			err := r.Get(ctx, *instance.GetCleanupJobMetadata(), jobQuery)
			if err != nil && errors.IsNotFound(err) {
				deleted = true
			}
			time.Sleep(time.Second * 1)
		}

		instance.Status.CleanupJobStatus = robotv1alpha1.OwnedJobStatus{}
	}

	return nil
}
