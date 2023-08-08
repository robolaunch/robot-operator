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
	goErr "errors"
	"time"

	batchv1 "k8s.io/api/batch/v1"
	corev1 "k8s.io/api/core/v1"
	"k8s.io/apimachinery/pkg/api/errors"
	"k8s.io/apimachinery/pkg/labels"
	"k8s.io/apimachinery/pkg/runtime"
	"k8s.io/apimachinery/pkg/selection"
	"k8s.io/apimachinery/pkg/types"
	"k8s.io/client-go/dynamic"
	ctrl "sigs.k8s.io/controller-runtime"
	"sigs.k8s.io/controller-runtime/pkg/client"
	"sigs.k8s.io/controller-runtime/pkg/handler"
	"sigs.k8s.io/controller-runtime/pkg/log"
	"sigs.k8s.io/controller-runtime/pkg/reconcile"
	"sigs.k8s.io/controller-runtime/pkg/source"

	"github.com/go-logr/logr"
	"github.com/robolaunch/robot-operator/internal"
	robotErr "github.com/robolaunch/robot-operator/internal/error"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
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

//+kubebuilder:rbac:groups=core,resources=configmaps,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=batch,resources=jobs,verbs=get;list;watch;create;update;patch;delete

var logger logr.Logger

func (r *BuildManagerReconciler) Reconcile(ctx context.Context, req ctrl.Request) (ctrl.Result, error) {
	logger = log.FromContext(ctx)

	var result ctrl.Result = ctrl.Result{}

	instance, err := r.reconcileGetInstance(ctx, req.NamespacedName)
	if err != nil {
		if errors.IsNotFound(err) {
			return ctrl.Result{}, nil
		}
		return ctrl.Result{}, err
	}

	// Check target robot's attached object, update activity status
	err = r.reconcileCheckTargetRobot(ctx, instance)
	if err != nil {
		if errors.IsNotFound(err) {
			instance.Status.Phase = robotv1alpha1.BuildManagerRobotNotFound
			instance.Status.Active = false
		} else {
			return ctrl.Result{}, err
		}
	}

	// Check target robot's other attached objects to see if robot's resources are released
	err = r.reconcileCheckOtherAttachedResources(ctx, instance)
	if err != nil {
		var e *robotErr.RobotResourcesHasNotBeenReleasedError
		if goErr.As(err, &e) {
			return ctrl.Result{
				Requeue:      true,
				RequeueAfter: 3 * time.Second,
			}, nil
		}
		return ctrl.Result{}, nil
	}

	err = r.reconcileCheckStatus(ctx, instance)
	if err != nil {
		var creatingResourceError *robotErr.CreatingResourceError
		var waitingForResourceError *robotErr.WaitingForResourceError
		if !(goErr.As(err, &creatingResourceError) || goErr.As(err, &waitingForResourceError)) {
			return ctrl.Result{}, err
		} else {
			result.Requeue = true
			result.RequeueAfter = 1 * time.Second
		}
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

		err := r.reconcileHandleConfigMap(ctx, instance)
		if err != nil {
			return err
		}

		err = r.reconcileHandleBuilderJobs(ctx, instance)
		if err != nil {
			return err
		}

		instance.Status.Phase = robotv1alpha1.BuildManagerReady

	case false:

		instance.Status.Phase = robotv1alpha1.BuildManagerDeactivating

		err := r.reconcileDeleteBuilderJobs(ctx, instance)
		if err != nil {
			return err
		}

		err = r.reconcileDeleteConfigMap(ctx, instance)
		if err != nil {
			return err
		}

		instance.Status.Phase = robotv1alpha1.BuildManagerInactive
	}

	return nil
}

func (r *BuildManagerReconciler) reconcileCheckResources(ctx context.Context, instance *robotv1alpha1.BuildManager) error {

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
		Watches(
			&source.Kind{Type: &robotv1alpha1.Robot{}},
			handler.EnqueueRequestsFromMapFunc(r.watchRobots),
		).
		Complete(r)
}

func (r *BuildManagerReconciler) watchRobots(o client.Object) []reconcile.Request {

	robot := o.(*robotv1alpha1.Robot)

	// Get attached build objects for this robot
	requirements := []labels.Requirement{}
	newReq, err := labels.NewRequirement(internal.TARGET_ROBOT_LABEL_KEY, selection.In, []string{robot.Name})
	if err != nil {
		return []reconcile.Request{}
	}
	requirements = append(requirements, *newReq)

	robotSelector := labels.NewSelector().Add(requirements...)

	buildManagerList := robotv1alpha1.BuildManagerList{}
	err = r.List(context.TODO(), &buildManagerList, &client.ListOptions{Namespace: robot.Namespace, LabelSelector: robotSelector})
	if err != nil {
		return []reconcile.Request{}
	}

	requests := make([]reconcile.Request, len(buildManagerList.Items))
	for i, item := range buildManagerList.Items {

		requests[i] = reconcile.Request{
			NamespacedName: types.NamespacedName{
				Name:      item.Name,
				Namespace: item.Namespace,
			},
		}

	}

	return requests
}
