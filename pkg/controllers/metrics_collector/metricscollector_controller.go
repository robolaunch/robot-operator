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

package metrics_collector

import (
	"context"

	"k8s.io/apimachinery/pkg/api/errors"
	"k8s.io/apimachinery/pkg/runtime"
	ctrl "sigs.k8s.io/controller-runtime"
	"sigs.k8s.io/controller-runtime/pkg/client"
	"sigs.k8s.io/controller-runtime/pkg/log"

	"github.com/go-logr/logr"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
)

// MetricsCollectorReconciler reconciles a MetricsCollector object
type MetricsCollectorReconciler struct {
	client.Client
	Scheme *runtime.Scheme
}

//+kubebuilder:rbac:groups=robot.roboscale.io,resources=metricscollectors,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=robot.roboscale.io,resources=metricscollectors/status,verbs=get;update;patch
//+kubebuilder:rbac:groups=robot.roboscale.io,resources=metricscollectors/finalizers,verbs=update

//+kubebuilder:rbac:groups=core,resources=pods/exec,verbs=get;list;watch;create;update;patch;delete

var logger logr.Logger

func (r *MetricsCollectorReconciler) Reconcile(ctx context.Context, req ctrl.Request) (ctrl.Result, error) {
	logger = log.FromContext(ctx)

	instance, err := r.reconcileGetInstance(ctx, req.NamespacedName)
	if err != nil {
		if errors.IsNotFound(err) {
			return ctrl.Result{}, nil
		}
		return ctrl.Result{}, err
	}

	err = r.reconcileCheckRobotRelatedPods(ctx, instance)
	if err != nil {
		return ctrl.Result{}, err
	}

	err = r.reconcileCheckUtilizations(ctx, instance)
	if err != nil {
		return ctrl.Result{}, err
	}

	err = r.reconcileUpdateInstanceStatus(ctx, instance)
	if err != nil {
		return ctrl.Result{}, err
	}

	return ctrl.Result{}, nil
}

func (r *MetricsCollectorReconciler) reconcileCheckRobotRelatedPods(ctx context.Context, instance *robotv1alpha1.MetricsCollector) error {

	err := r.reconcileGetPods(ctx, instance)
	if err != nil {
		return err
	}

	return nil
}

func (r *MetricsCollectorReconciler) reconcileCheckUtilizations(ctx context.Context, instance *robotv1alpha1.MetricsCollector) error {
	return nil
}

// SetupWithManager sets up the controller with the Manager.
func (r *MetricsCollectorReconciler) SetupWithManager(mgr ctrl.Manager) error {
	return ctrl.NewControllerManagedBy(mgr).
		For(&robotv1alpha1.MetricsCollector{}).
		Complete(r)
}