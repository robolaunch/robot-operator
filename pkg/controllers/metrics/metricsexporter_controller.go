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

package metrics

import (
	"context"

	corev1 "k8s.io/api/core/v1"
	rbacv1 "k8s.io/api/rbac/v1"
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
	robotErr "github.com/robolaunch/robot-operator/internal/error"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
)

// MetricsExporterReconciler reconciles a MetricsExporter object
type MetricsExporterReconciler struct {
	client.Client
	Scheme *runtime.Scheme
}

//+kubebuilder:rbac:groups=robot.roboscale.io,resources=metricsexporters,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=robot.roboscale.io,resources=metricsexporters/status,verbs=get;update;patch
//+kubebuilder:rbac:groups=robot.roboscale.io,resources=metricsexporters/finalizers,verbs=update

//+kubebuilder:rbac:groups=core,resources=nodes,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=core,resources=pods,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=core,resources=serviceaccounts,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=rbac.authorization.k8s.io,resources=roles,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=rbac.authorization.k8s.io,resources=rolebindings,verbs=get;list;watch;create;update;patch;delete

var logger logr.Logger

func (r *MetricsExporterReconciler) Reconcile(ctx context.Context, req ctrl.Request) (ctrl.Result, error) {
	logger = log.FromContext(ctx)

	var result ctrl.Result = ctrl.Result{}

	instance, err := r.reconcileGetInstance(ctx, req.NamespacedName)
	if err != nil {
		if errors.IsNotFound(err) {
			return ctrl.Result{}, nil
		}
		return ctrl.Result{}, err
	}

	if !instance.DeletionTimestamp.IsZero() {
		return ctrl.Result{}, nil
	}

	err = r.reconcileCheckGPUCapacities(ctx, instance)
	if err != nil {
		return result, err
	}

	err = r.reconcileCheckGPUConsumingPods(ctx, instance)
	if err != nil {
		return result, err
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

	return ctrl.Result{}, nil
}

func (r *MetricsExporterReconciler) reconcileCheckStatus(ctx context.Context, instance *robotv1alpha1.MetricsExporter, result *ctrl.Result) error {

	if instance.Spec.GPU.Track || instance.Spec.Network.Track {

		err := r.reconcileHandleRole(ctx, instance)
		if err != nil {
			return robotErr.CheckCreatingOrWaitingError(result, err)
		}

		err = r.reconcileHandleServiceAccount(ctx, instance)
		if err != nil {
			return robotErr.CheckCreatingOrWaitingError(result, err)
		}

		err = r.reconcileHandleRoleBinding(ctx, instance)
		if err != nil {
			return robotErr.CheckCreatingOrWaitingError(result, err)
		}

		err = r.reconcileHandlePod(ctx, instance)
		if err != nil {
			return robotErr.CheckCreatingOrWaitingError(result, err)
		}

		instance.Status.Phase = robotv1alpha1.MetricsExporterPhaseReady
	}

	return nil
}

func (r *MetricsExporterReconciler) reconcileCheckResources(ctx context.Context, instance *robotv1alpha1.MetricsExporter) error {

	err := r.reconcileCheckRole(ctx, instance)
	if err != nil {
		return err
	}

	err = r.reconcileCheckServiceAccount(ctx, instance)
	if err != nil {
		return err
	}

	err = r.reconcileCheckRoleBinding(ctx, instance)
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
func (r *MetricsExporterReconciler) SetupWithManager(mgr ctrl.Manager) error {
	return ctrl.NewControllerManagedBy(mgr).
		For(&robotv1alpha1.MetricsExporter{}).
		Owns(&rbacv1.Role{}).
		Owns(&rbacv1.RoleBinding{}).
		Owns(&corev1.ServiceAccount{}).
		Owns(&corev1.Pod{}).
		Watches(
			&source.Kind{Type: &corev1.Node{}},
			handler.EnqueueRequestsFromMapFunc(r.watchNodes),
		).
		Complete(r)
}

func (r *MetricsExporterReconciler) watchNodes(o client.Object) []reconcile.Request {

	obj := o.(*corev1.Node)

	metricsExporterList := &robotv1alpha1.MetricsExporterList{}
	err := r.List(context.TODO(), metricsExporterList)
	if err != nil {
		return []reconcile.Request{}
	}

	requests := []reconcile.Request{}

	for _, me := range metricsExporterList.Items {
		activeNode, err := r.reconcileCheckNode(context.TODO(), &me)
		if err != nil {
			return []reconcile.Request{}
		}
		if obj.Name == activeNode.Name {
			requests = append(requests, reconcile.Request{
				NamespacedName: types.NamespacedName{
					Name:      me.Name,
					Namespace: me.Namespace,
				},
			})
		}
	}

	return requests
}
