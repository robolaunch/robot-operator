/*
Copyright 2022.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

package workspace_manager

import (
	"context"

	batchv1 "k8s.io/api/batch/v1"
	"k8s.io/apimachinery/pkg/api/errors"
	"k8s.io/apimachinery/pkg/runtime"
	ctrl "sigs.k8s.io/controller-runtime"
	"sigs.k8s.io/controller-runtime/pkg/client"
	"sigs.k8s.io/controller-runtime/pkg/log"

	"github.com/go-logr/logr"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
)

// WorkspaceManagerReconciler reconciles a WorkspaceManager object
type WorkspaceManagerReconciler struct {
	client.Client
	Scheme *runtime.Scheme
}

//+kubebuilder:rbac:groups=robot.roboscale.io,resources=workspacemanagers,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=robot.roboscale.io,resources=workspacemanagers/status,verbs=get;update;patch
//+kubebuilder:rbac:groups=robot.roboscale.io,resources=workspacemanagers/finalizers,verbs=update

//+kubebuilder:rbac:groups=batch,resources=jobs,verbs=get;list;watch;create;update;patch;delete

var logger logr.Logger

func (r *WorkspaceManagerReconciler) Reconcile(ctx context.Context, req ctrl.Request) (ctrl.Result, error) {
	logger = log.FromContext(ctx)

	instance, err := r.reconcileGetInstance(ctx, req.NamespacedName)
	if err != nil {
		if errors.IsNotFound(err) {
			return ctrl.Result{}, nil
		}
		return ctrl.Result{}, err
	}

	err = r.reconcileCheckStatus(ctx, instance)
	if err != nil {
		return ctrl.Result{}, err
	}

	err = r.reconcileUpdateInstanceStatus(ctx, instance)
	if err != nil {
		return ctrl.Result{}, err
	}

	err = r.reconcileCheckResources(ctx, instance)
	if err != nil {
		return ctrl.Result{}, err
	}

	err = r.reconcileUpdateInstanceStatus(ctx, instance)
	if err != nil {
		return ctrl.Result{}, err
	}

	return ctrl.Result{}, nil
}

func (r *WorkspaceManagerReconciler) reconcileCheckStatus(ctx context.Context, instance *robotv1alpha1.WorkspaceManager) error {

	switch instance.Spec.UpdateNeeded {
	case true:

		err := r.reconcileDeleteClonerJob(ctx, instance)
		if err != nil {
			return err
		}

		instance.Spec.UpdateNeeded = false
		err = r.Update(ctx, instance, &client.UpdateOptions{})
		if err != nil {
			return err
		}

		instance.Status.Version++
		instance.Status.Phase = robotv1alpha1.WorkspaceManagerPhaseConfiguringWorkspaces

	}

	switch instance.Status.ClonerJobStatus.Created {
	case true:

		switch instance.Status.ClonerJobStatus.Phase {
		case robotv1alpha1.JobSucceeded:

			instance.Status.Phase = robotv1alpha1.WorkspaceManagerPhaseReady

		case robotv1alpha1.JobActive:

			instance.Status.Phase = robotv1alpha1.WorkspaceManagerPhaseConfiguringWorkspaces

		case robotv1alpha1.JobFailed:

			instance.Status.Phase = robotv1alpha1.WorkspaceManagerPhaseFailed

		}

	case false:

		instance.Status.Phase = robotv1alpha1.WorkspaceManagerPhaseConfiguringWorkspaces
		err := r.createJob(ctx, instance, instance.GetClonerJobMetadata())
		if err != nil {
			return err
		}
		instance.Status.ClonerJobStatus.Created = true

	}

	return nil
}

func (r *WorkspaceManagerReconciler) reconcileCheckResources(ctx context.Context, instance *robotv1alpha1.WorkspaceManager) error {

	err := r.reconcileCheckClonerJob(ctx, instance)
	if err != nil {
		return err
	}

	return nil
}

// SetupWithManager sets up the controller with the Manager.
func (r *WorkspaceManagerReconciler) SetupWithManager(mgr ctrl.Manager) error {
	return ctrl.NewControllerManagedBy(mgr).
		For(&robotv1alpha1.WorkspaceManager{}).
		Owns(&batchv1.Job{}).
		Complete(r)
}
