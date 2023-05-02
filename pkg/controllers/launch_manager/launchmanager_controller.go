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

package launch_manager

import (
	"context"

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
	"github.com/robolaunch/robot-operator/internal/hybrid"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
)

// LaunchManagerReconciler reconciles a LaunchManager object
type LaunchManagerReconciler struct {
	client.Client
	Scheme        *runtime.Scheme
	DynamicClient dynamic.Interface
}

//+kubebuilder:rbac:groups=robot.roboscale.io,resources=launchmanagers,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=robot.roboscale.io,resources=launchmanagers/status,verbs=get;update;patch
//+kubebuilder:rbac:groups=robot.roboscale.io,resources=launchmanagers/finalizers,verbs=update

var logger logr.Logger

func (r *LaunchManagerReconciler) Reconcile(ctx context.Context, req ctrl.Request) (ctrl.Result, error) {
	logger = log.FromContext(ctx)

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
			instance.Status.Phase = robotv1alpha1.LaunchManagerPhaseRobotNotFound
			instance.Status.Active = false
		} else {
			return ctrl.Result{}, err
		}
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

func (r *LaunchManagerReconciler) reconcileCheckStatus(ctx context.Context, instance *robotv1alpha1.LaunchManager) error {

	switch instance.Status.Active {
	case true:

		robot, err := r.reconcileGetTargetRobot(ctx, instance)
		if err != nil {
			return err
		}

		switch hybrid.HasLaunchInThisInstance(*instance, *robot) {
		case true:

			switch instance.Status.LaunchPodStatus.Status.Resource.Created {
			case true:

				switch instance.Status.LaunchPodStatus.Status.Resource.Phase {
				case string(corev1.PodRunning):

					instance.Status.Phase = robotv1alpha1.LaunchManagerPhaseReady

				}

			case false:

				instance.Status.Phase = robotv1alpha1.LaunchManagerPhaseCreatingPod
				err := r.createLaunchPod(ctx, instance)
				if err != nil {
					return err
				}
				instance.Status.LaunchPodStatus.Status.Resource.Created = true
				instance.Status.Phase = robotv1alpha1.LaunchManagerPhaseLaunching

			}

		case false:

			instance.Status.Phase = robotv1alpha1.LaunchManagerPhaseReady

		}

	case false:

		instance.Status.Phase = robotv1alpha1.LaunchManagerPhaseDeactivating

		err := r.reconcileDeleteLaunchPod(ctx, instance)
		if err != nil {
			return err
		}

		instance.Status.Phase = robotv1alpha1.LaunchManagerPhaseInactive

	}

	return nil
}

func (r *LaunchManagerReconciler) reconcileCheckResources(ctx context.Context, instance *robotv1alpha1.LaunchManager) error {

	err := r.reconcileCheckLaunchPod(ctx, instance)
	if err != nil {
		return err
	}

	return nil
}

// SetupWithManager sets up the controller with the Manager.
func (r *LaunchManagerReconciler) SetupWithManager(mgr ctrl.Manager) error {
	return ctrl.NewControllerManagedBy(mgr).
		For(&robotv1alpha1.LaunchManager{}).
		Owns(&corev1.Pod{}).
		Watches(
			&source.Kind{Type: &robotv1alpha1.Robot{}},
			handler.EnqueueRequestsFromMapFunc(r.watchRobots),
		).
		Complete(r)
}

func (r *LaunchManagerReconciler) watchRobots(o client.Object) []reconcile.Request {

	robot := o.(*robotv1alpha1.Robot)

	// Get attached build objects for this robot
	requirements := []labels.Requirement{}
	newReq, err := labels.NewRequirement(internal.TARGET_ROBOT_LABEL_KEY, selection.In, []string{robot.Name})
	if err != nil {
		return []reconcile.Request{}
	}
	requirements = append(requirements, *newReq)

	robotSelector := labels.NewSelector().Add(requirements...)

	launchManagerList := robotv1alpha1.LaunchManagerList{}
	err = r.List(context.TODO(), &launchManagerList, &client.ListOptions{Namespace: robot.Namespace, LabelSelector: robotSelector})
	if err != nil {
		return []reconcile.Request{}
	}

	requests := make([]reconcile.Request, len(launchManagerList.Items))
	for i, item := range launchManagerList.Items {

		requests[i] = reconcile.Request{
			NamespacedName: types.NamespacedName{
				Name:      item.Name,
				Namespace: item.Namespace,
			},
		}

	}

	return requests
}
