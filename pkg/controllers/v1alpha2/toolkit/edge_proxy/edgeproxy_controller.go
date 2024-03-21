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

	"github.com/robolaunch/robot-operator/internal"
	robotErr "github.com/robolaunch/robot-operator/internal/error"
	appsv1 "k8s.io/api/apps/v1"
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

//+kubebuilder:rbac:groups=apps,resources=deployments,verbs=get;list;watch;create;update;patch;delete

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

	err := r.reconcileHandleDeployment(ctx, instance)
	if err != nil {
		return robotErr.CheckCreatingOrWaitingError(result, err)
	}

	return nil
}

func (r *EdgeProxyReconciler) reconcileCheckResources(ctx context.Context, instance *robotv1alpha2.EdgeProxy) error {

	err := r.reconcileCheckDeployment(ctx, instance)
	if err != nil {
		return err
	}

	err = r.reconcileCalculatePhase(ctx, instance)
	if err != nil {
		return err
	}

	return nil
}

func (r *EdgeProxyReconciler) reconcileCalculatePhase(ctx context.Context, instance *robotv1alpha2.EdgeProxy) error {

	containersReady := true
	if len(instance.Status.DeploymentStatus.ContainerStatuses) > 0 {
		for _, cStatus := range instance.Status.DeploymentStatus.ContainerStatuses {
			containersReady = containersReady && cStatus.Ready
		}
	} else {
		containersReady = false
	}

	if containersReady && instance.Status.ServiceStatus.Resource.Created && (instance.Spec.Ingress == instance.Status.IngressStatus.Created) {
		if edgeProxyURL, ok := instance.Status.ServiceStatus.URLs[internal.EDGE_PROXY_APP_NAME]; ok && instance.Status.Phase != robotv1alpha2.EdgeProxyPhaseReady {
			r.Recorder.Event(instance, "Normal", "Ready", "EdgeProxy service is accessible over the URL '"+edgeProxyURL+"'.")
		}
		instance.Status.Phase = robotv1alpha2.EdgeProxyPhaseReady
	} else {
		instance.Status.Phase = robotv1alpha2.EdgeProxyPhaseConfiguringResources
	}

	return nil
}

// SetupWithManager sets up the controller with the Manager.
func (r *EdgeProxyReconciler) SetupWithManager(mgr ctrl.Manager) error {
	return ctrl.NewControllerManagedBy(mgr).
		For(&robotv1alpha2.EdgeProxy{}).
		Owns(&appsv1.Deployment{}).
		Complete(r)
}
