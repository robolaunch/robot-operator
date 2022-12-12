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

package build_manager

import (
	"context"
	"sort"

	batchv1 "k8s.io/api/batch/v1"
	corev1 "k8s.io/api/core/v1"
	"k8s.io/apimachinery/pkg/api/errors"
	"k8s.io/apimachinery/pkg/runtime"
	"k8s.io/client-go/dynamic"
	ctrl "sigs.k8s.io/controller-runtime"
	"sigs.k8s.io/controller-runtime/pkg/client"
	"sigs.k8s.io/controller-runtime/pkg/log"

	"github.com/go-logr/logr"
	robotv1alpha1 "github.com/robolaunch/robot-operator/api/v1alpha1"
)

// BuildManagerReconciler reconciles a BuildManager object
type BuildManagerReconciler struct {
	client.Client
	Scheme        *runtime.Scheme
	DynamicClient dynamic.Interface
}

//+kubebuilder:rbac:groups=robot.roboscale.io,resources=buildmanagers,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=robot.roboscale.io,resources=buildmanagers/status,verbs=get;update;patch
//+kubebuilder:rbac:groups=robot.roboscale.io,resources=buildmanagers/finalizers,verbs=update

//+kubebuilder:rbac:groups=batch,resources=jobs,verbs=get;list;watch;create;update;patch;delete

var logger logr.Logger
var defaultReturnResult ctrl.Result

func (r *BuildManagerReconciler) Reconcile(ctx context.Context, req ctrl.Request) (ctrl.Result, error) {
	logger = log.FromContext(ctx)
	defaultReturnResult = ctrl.Result{}

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

func (r *BuildManagerReconciler) reconcileCheckStatus(ctx context.Context, instance *robotv1alpha1.BuildManager) error {
	switch instance.Status.Active {
	case true:

		switch instance.Status.ScriptConfigMapStatus.Created {
		case true:

			instance.Status.Phase = robotv1alpha1.BuildManagerBuildingRobot

			keys := []string{}
			for key := range instance.Status.Steps {
				keys = append(keys, key)
			}

			sort.Strings(keys)

			for _, k := range keys {
				if instance.Status.Steps[k].JobCreated {

					if instance.Status.Steps[k].JobPhase == robotv1alpha1.JobSucceeded {
						continue
					} else if instance.Status.Steps[k].JobPhase == robotv1alpha1.JobActive {
						Requeue(&defaultReturnResult)
						break
					} else if instance.Status.Steps[k].JobPhase == robotv1alpha1.JobFailed {
						Requeue(&defaultReturnResult)
						break
					} else {
						Requeue(&defaultReturnResult)
						break
					}

				} else {

					err := r.createBuilderJob(ctx, instance, k)
					if err != nil {
						return err
					}

					break
				}
			}

			areJobsSucceeded := false

			for _, key := range keys {
				if instance.Status.Steps[key].JobPhase == robotv1alpha1.JobSucceeded {
					areJobsSucceeded = true
				} else {
					areJobsSucceeded = false
					break
				}
			}

			if areJobsSucceeded {
				instance.Status.Phase = robotv1alpha1.BuildManagerReady
			}

		case false:

			instance.Status.Phase = robotv1alpha1.BuildManagerCreatingConfigMap
			err := r.createScriptConfigMap(ctx, instance)
			if err != nil {
				return err
			}
			instance.Status.ScriptConfigMapStatus.Created = true

		}

	case false:

		// remove workloads if any
		// do nothing

	}

	return nil
}

func (r *BuildManagerReconciler) reconcileCheckResources(ctx context.Context, instance *robotv1alpha1.BuildManager) error {

	phase := instance.Status.Phase
	instance.Status = robotv1alpha1.BuildManagerStatus{}
	instance.Status.Phase = phase

	err := r.reconcileCheckConfigMap(ctx, instance)
	if err != nil {
		return err
	}

	err = r.reconcileCheckBuilderJobs(ctx, instance)
	if err != nil {
		return err
	}

	return nil
}

// SetupWithManager sets up the controller with the Manager.
func (r *BuildManagerReconciler) SetupWithManager(mgr ctrl.Manager) error {
	return ctrl.NewControllerManagedBy(mgr).
		For(&robotv1alpha1.BuildManager{}).
		Owns(&corev1.ConfigMap{}).
		Owns(&batchv1.Job{}).
		Complete(r)
}
