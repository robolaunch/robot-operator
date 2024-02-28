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

package ros2_bridge

import (
	"context"

	robotErr "github.com/robolaunch/robot-operator/internal/error"
	corev1 "k8s.io/api/core/v1"
	networkingv1 "k8s.io/api/networking/v1"
	"k8s.io/apimachinery/pkg/api/errors"
	"k8s.io/apimachinery/pkg/runtime"
	"k8s.io/apimachinery/pkg/types"
	ctrl "sigs.k8s.io/controller-runtime"
	"sigs.k8s.io/controller-runtime/pkg/client"
	"sigs.k8s.io/controller-runtime/pkg/handler"
	"sigs.k8s.io/controller-runtime/pkg/log"
	"sigs.k8s.io/controller-runtime/pkg/reconcile"
	"sigs.k8s.io/controller-runtime/pkg/source"

	"github.com/go-logr/logr"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	robotv1alpha2 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha2"
)

// ROS2BridgeReconciler reconciles a ROS2Bridge object
type ROS2BridgeReconciler struct {
	client.Client
	Scheme *runtime.Scheme
}

//+kubebuilder:rbac:groups=robot.roboscale.io,resources=ros2bridges,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=robot.roboscale.io,resources=ros2bridges/status,verbs=get;update;patch
//+kubebuilder:rbac:groups=robot.roboscale.io,resources=ros2bridges/finalizers,verbs=update

//+kubebuilder:rbac:groups=core,resources=pods,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=core,resources=services,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=networking.k8s.io,resources=ingresses,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=robot.roboscale.io,resources=discoveryservers,verbs=get;list;watch

var logger logr.Logger

func (r *ROS2BridgeReconciler) Reconcile(ctx context.Context, req ctrl.Request) (ctrl.Result, error) {
	logger = log.FromContext(ctx)

	var result ctrl.Result = ctrl.Result{}

	instance, err := r.reconcileGetInstance(ctx, req.NamespacedName)
	if err != nil {
		if errors.IsNotFound(err) {
			return ctrl.Result{}, nil
		}
		return ctrl.Result{}, err
	}

	err = r.reconcileCheckStatus(ctx, instance, &result)
	if err != nil {
		return result, err
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
	return result, nil
}

func (r *ROS2BridgeReconciler) reconcileCheckStatus(ctx context.Context, instance *robotv1alpha2.ROS2Bridge, result *ctrl.Result) error {

	err := r.reconcileHandleConnectionInfo(ctx, instance)
	if err != nil {
		return robotErr.CheckCreatingOrWaitingError(result, err)
	}

	err = r.reconcileHandleService(ctx, instance)
	if err != nil {
		return robotErr.CheckCreatingOrWaitingError(result, err)
	}

	err = r.reconcileHandlePod(ctx, instance)
	if err != nil {
		return robotErr.CheckCreatingOrWaitingError(result, err)
	}

	err = r.reconcileHandleIngress(ctx, instance)
	if err != nil {
		return robotErr.CheckCreatingOrWaitingError(result, err)
	}

	instance.Status.Phase = robotv1alpha2.ROS2BridgePhaseReady

	return nil
}

func (r *ROS2BridgeReconciler) reconcileCheckResources(ctx context.Context, instance *robotv1alpha2.ROS2Bridge) error {

	err := r.reconcileCheckService(ctx, instance)
	if err != nil {
		return err
	}

	err = r.reconcileCheckPod(ctx, instance)
	if err != nil {
		return err
	}

	err = r.reconcileCheckIngress(ctx, instance)
	if err != nil {
		return err
	}

	return nil
}

// SetupWithManager sets up the controller with the Manager.
func (r *ROS2BridgeReconciler) SetupWithManager(mgr ctrl.Manager) error {
	return ctrl.NewControllerManagedBy(mgr).
		For(&robotv1alpha2.ROS2Bridge{}).
		Owns(&corev1.Pod{}).
		Owns(&corev1.Service{}).
		Owns(&networkingv1.Ingress{}).
		Watches(
			&source.Kind{Type: &robotv1alpha1.DiscoveryServer{}},
			handler.EnqueueRequestsFromMapFunc(r.watchDiscoveryServer),
		).
		Complete(r)
}

func (r *ROS2BridgeReconciler) watchDiscoveryServer(o client.Object) []reconcile.Request {

	obj := o.(*robotv1alpha1.DiscoveryServer)

	ros2bridgeList := &robotv1alpha2.ROS2BridgeList{}
	err := r.List(context.TODO(), ros2bridgeList)
	if err != nil {
		return []reconcile.Request{}
	}

	requests := []reconcile.Request{}
	for _, r2b := range ros2bridgeList.Items {
		if r2b.Spec.DiscoveryServerReference.Name == obj.Name && r2b.Spec.DiscoveryServerReference.Namespace == obj.Namespace {
			requests = append(requests, reconcile.Request{
				NamespacedName: types.NamespacedName{
					Namespace: r2b.Namespace,
					Name:      r2b.Name,
				},
			})
		}
	}

	return requests
}
