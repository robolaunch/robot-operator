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

package ros_bridge

import (
	"context"

	corev1 "k8s.io/api/core/v1"
	"k8s.io/apimachinery/pkg/api/errors"
	"k8s.io/apimachinery/pkg/runtime"
	"k8s.io/client-go/dynamic"
	ctrl "sigs.k8s.io/controller-runtime"
	"sigs.k8s.io/controller-runtime/pkg/client"
	"sigs.k8s.io/controller-runtime/pkg/log"

	"github.com/go-logr/logr"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
)

// ROSBridgeReconciler reconciles a ROSBridge object
type ROSBridgeReconciler struct {
	client.Client
	Scheme        *runtime.Scheme
	DynamicClient dynamic.Interface
}

//+kubebuilder:rbac:groups=robot.roboscale.io,resources=rosbridges,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=robot.roboscale.io,resources=rosbridges/status,verbs=get;update;patch
//+kubebuilder:rbac:groups=robot.roboscale.io,resources=rosbridges/finalizers,verbs=update

//+kubebuilder:rbac:groups=core,resources=pods,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=core,resources=services,verbs=get;list;watch;create;update;patch;delete

var logger logr.Logger

func (r *ROSBridgeReconciler) Reconcile(ctx context.Context, req ctrl.Request) (ctrl.Result, error) {
	logger = log.FromContext(ctx)

	instance, err := r.reconcileGetInstance(ctx, req.NamespacedName)
	if err != nil {
		if errors.IsNotFound(err) {
			return ctrl.Result{}, nil
		}
		return ctrl.Result{}, err
	}

	// err = r.reconcileCheckDeletion(ctx, instance)
	// if err != nil {

	// 	if errors.IsNotFound(err) {
	// 		return ctrl.Result{}, nil
	// 	}

	// 	return ctrl.Result{}, err
	// }

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

func (r *ROSBridgeReconciler) reconcileCheckStatus(ctx context.Context, instance *robotv1alpha1.ROSBridge) error {

	switch instance.Status.ServiceStatus.Created {
	case true:

		switch instance.Status.PodStatus.Resource.Created {
		case true:

			switch instance.Status.PodStatus.Phase {
			case corev1.PodRunning:

				// TODO: handle other pod phases

				instance.Status.Phase = robotv1alpha1.BridgePhaseReady

			}

		case false:

			instance.Status.Phase = robotv1alpha1.BridgePhaseCreatingPod
			err := r.createPod(ctx, instance, instance.GetBridgePodMetadata())
			if err != nil {
				return err
			}
			instance.Status.PodStatus.Resource.Created = true

		}

	case false:

		instance.Status.Phase = robotv1alpha1.BridgePhaseCreatingService
		err := r.createService(ctx, instance, instance.GetBridgeServiceMetadata())
		if err != nil {
			return err
		}
		instance.Status.ServiceStatus.Created = true

	}

	return nil
}

func (r *ROSBridgeReconciler) reconcileCheckResources(ctx context.Context, instance *robotv1alpha1.ROSBridge) error {

	err := r.reconcileCheckService(ctx, instance)
	if err != nil {
		return err
	}

	err = r.reconcileCheckPod(ctx, instance)
	if err != nil {
		return err
	}

	return nil
}

// SetupWithManager sets up the controller with the Manager.
func (r *ROSBridgeReconciler) SetupWithManager(mgr ctrl.Manager) error {
	return ctrl.NewControllerManagedBy(mgr).
		For(&robotv1alpha1.ROSBridge{}).
		Owns(&corev1.Pod{}).
		Owns(&corev1.Service{}).
		Complete(r)
}
