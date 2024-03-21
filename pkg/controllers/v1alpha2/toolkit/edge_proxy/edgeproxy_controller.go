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

package edge_proxy

import (
	"context"

	"k8s.io/apimachinery/pkg/api/errors"
	"k8s.io/apimachinery/pkg/runtime"
	"k8s.io/client-go/tools/record"
	ctrl "sigs.k8s.io/controller-runtime"
	"sigs.k8s.io/controller-runtime/pkg/client"
	"sigs.k8s.io/controller-runtime/pkg/log"

	"github.com/go-logr/logr"
	robotv1alpha2 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha2"
)

// EdgeProxyReconciler reconciles a EdgeProxy object
type EdgeProxyReconciler struct {
	client.Client
	Scheme   *runtime.Scheme
	Recorder record.EventRecorder
}

//+kubebuilder:rbac:groups=robot.roboscale.io,resources=edgeproxies,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=robot.roboscale.io,resources=edgeproxies/status,verbs=get;update;patch
//+kubebuilder:rbac:groups=robot.roboscale.io,resources=edgeproxies/finalizers,verbs=update

var logger logr.Logger

func (r *EdgeProxyReconciler) Reconcile(ctx context.Context, req ctrl.Request) (ctrl.Result, error) {
	logger = log.FromContext(ctx)

	var result ctrl.Result = ctrl.Result{}

	instance, err := r.reconcileGetInstance(ctx, req.NamespacedName)
	if err != nil {
		if errors.IsNotFound(err) {
			return ctrl.Result{}, nil
		}
		return ctrl.Result{}, err
	}

	err = r.reconcileRegisterResources(ctx, instance)
	if err != nil {
		return result, err
	}

	err = r.reconcileUpdateInstanceStatus(ctx, instance)
	if err != nil {
		return result, err
	}

	err = r.reconcileCheckStatus(ctx, instance, &result)
	if err != nil {
		return result, err
	}

	err = r.reconcileUpdateInstanceStatus(ctx, instance)
	if err != nil {
		return result, err
	}

	err = r.reconcileCheckResources(ctx, instance)
	if err != nil {
		return result, err
	}

	err = r.reconcileUpdateInstanceStatus(ctx, instance)
	if err != nil {
		return result, err
	}

	return result, nil
}

func (r *EdgeProxyReconciler) reconcileRegisterResources(ctx context.Context, instance *robotv1alpha2.EdgeProxy) error {
	return nil
}

func (r *EdgeProxyReconciler) reconcileCheckStatus(ctx context.Context, instance *robotv1alpha2.EdgeProxy, result *ctrl.Result) error {
	return nil
}

func (r *EdgeProxyReconciler) reconcileCheckResources(ctx context.Context, instance *robotv1alpha2.EdgeProxy) error {
	return nil
}

func (r *EdgeProxyReconciler) reconcileCalculatePhase(ctx context.Context, instance *robotv1alpha2.EdgeProxy) error {
	return nil
}

// SetupWithManager sets up the controller with the Manager.
func (r *EdgeProxyReconciler) SetupWithManager(mgr ctrl.Manager) error {
	return ctrl.NewControllerManagedBy(mgr).
		For(&robotv1alpha2.EdgeProxy{}).
		Complete(r)
}
