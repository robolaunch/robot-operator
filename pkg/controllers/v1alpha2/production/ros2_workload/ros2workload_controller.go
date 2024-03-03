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

package ros2_workload

import (
	"context"

	robotErr "github.com/robolaunch/robot-operator/internal/error"
	corev1 "k8s.io/api/core/v1"
	"k8s.io/apimachinery/pkg/api/errors"
	"k8s.io/apimachinery/pkg/runtime"
	ctrl "sigs.k8s.io/controller-runtime"
	"sigs.k8s.io/controller-runtime/pkg/client"
	"sigs.k8s.io/controller-runtime/pkg/log"

	"github.com/go-logr/logr"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	robotv1alpha2 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha2"
)

// ROS2WorkloadReconciler reconciles a ROS2Workload object
type ROS2WorkloadReconciler struct {
	client.Client
	Scheme *runtime.Scheme
}

//+kubebuilder:rbac:groups=robot.roboscale.io,resources=ros2workloads,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=robot.roboscale.io,resources=ros2workloads/status,verbs=get;update;patch
//+kubebuilder:rbac:groups=robot.roboscale.io,resources=ros2workloads/finalizers,verbs=update

//+kubebuilder:rbac:groups=robot.roboscale.io,resources=discoveryservers,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=robot.roboscale.io,resources=ros2bridges,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=core,resources=persistentvolumeclaims,verbs=get;list;watch;create;update;patch;delete

var logger logr.Logger

func (r *ROS2WorkloadReconciler) Reconcile(ctx context.Context, req ctrl.Request) (ctrl.Result, error) {
	logger = log.FromContext(ctx)

	var result ctrl.Result = ctrl.Result{}

	instance, err := r.reconcileGetInstance(ctx, req.NamespacedName)
	if err != nil {
		if errors.IsNotFound(err) {
			return ctrl.Result{}, nil
		}
		return ctrl.Result{}, err
	}

	r.reconcileRegisterResources(instance)

	err = r.reconcileUpdateInstanceStatus(ctx, instance)
	if err != nil {
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

func (r *ROS2WorkloadReconciler) reconcileRegisterResources(instance *robotv1alpha2.ROS2Workload) error {

	r.registerPVCs(instance)

	return nil
}

func (r *ROS2WorkloadReconciler) reconcileCheckStatus(ctx context.Context, instance *robotv1alpha2.ROS2Workload, result *ctrl.Result) error {

	err := r.reconcileHandleDiscoveryServer(ctx, instance)
	if err != nil {
		return robotErr.CheckCreatingOrWaitingError(result, err)
	}

	err = r.reconcileHandleROS2Bridge(ctx, instance)
	if err != nil {
		return robotErr.CheckCreatingOrWaitingError(result, err)
	}

	err = r.reconcileHandlePVCs(ctx, instance)
	if err != nil {
		return robotErr.CheckCreatingOrWaitingError(result, err)
	}

	return nil
}

func (r *ROS2WorkloadReconciler) reconcileCheckResources(ctx context.Context, instance *robotv1alpha2.ROS2Workload) error {

	err := r.reconcileCheckDiscoveryServer(ctx, instance)
	if err != nil {
		return err
	}

	err = r.reconcileCheckROS2Bridge(ctx, instance)
	if err != nil {
		return err
	}

	err = r.reconcileCheckPVCs(ctx, instance)
	if err != nil {
		return err
	}

	return nil
}

// SetupWithManager sets up the controller with the Manager.
func (r *ROS2WorkloadReconciler) SetupWithManager(mgr ctrl.Manager) error {
	return ctrl.NewControllerManagedBy(mgr).
		For(&robotv1alpha2.ROS2Workload{}).
		Owns(&robotv1alpha1.DiscoveryServer{}).
		Owns(&robotv1alpha2.ROS2Bridge{}).
		Owns(&corev1.PersistentVolumeClaim{}).
		Complete(r)
}
