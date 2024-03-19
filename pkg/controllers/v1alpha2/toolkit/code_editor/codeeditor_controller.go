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

package code_editor

import (
	"context"

	robotErr "github.com/robolaunch/robot-operator/internal/error"
	appsv1 "k8s.io/api/apps/v1"
	corev1 "k8s.io/api/core/v1"
	networkingv1 "k8s.io/api/networking/v1"
	"k8s.io/apimachinery/pkg/api/errors"
	"k8s.io/apimachinery/pkg/runtime"
	"k8s.io/client-go/tools/record"
	ctrl "sigs.k8s.io/controller-runtime"
	"sigs.k8s.io/controller-runtime/pkg/client"
	"sigs.k8s.io/controller-runtime/pkg/log"

	"github.com/go-logr/logr"
	robotv1alpha2 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha2"
)

// CodeEditorReconciler reconciles a CodeEditor object
type CodeEditorReconciler struct {
	client.Client
	Scheme   *runtime.Scheme
	Recorder record.EventRecorder
}

//+kubebuilder:rbac:groups=robot.roboscale.io,resources=codeeditors,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=robot.roboscale.io,resources=codeeditors/status,verbs=get;update;patch
//+kubebuilder:rbac:groups=robot.roboscale.io,resources=codeeditors/finalizers,verbs=update

//+kubebuilder:rbac:groups=core,resources=persistentvolumeclaims,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=apps,resources=deployments,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=core,resources=services,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=networking.k8s.io,resources=ingresses,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=core,resources=events,verbs=get;list;watch;create;update;patch;delete

var logger logr.Logger

func (r *CodeEditorReconciler) Reconcile(ctx context.Context, req ctrl.Request) (ctrl.Result, error) {
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

func (r *CodeEditorReconciler) reconcileRegisterResources(ctx context.Context, instance *robotv1alpha2.CodeEditor) error {

	err := r.registerPVCs(ctx, instance)
	if err != nil {
		return err
	}

	err = r.registerExternalVolumes(ctx, instance)
	if err != nil {
		return err
	}

	return nil
}

func (r *CodeEditorReconciler) reconcileCheckStatus(ctx context.Context, instance *robotv1alpha2.CodeEditor, result *ctrl.Result) error {

	err := r.reconcileHandlePVCs(ctx, instance)
	if err != nil {
		return robotErr.CheckCreatingOrWaitingError(result, err)
	}

	err = r.reconcileHandleDeployment(ctx, instance)
	if err != nil {
		return robotErr.CheckCreatingOrWaitingError(result, err)
	}

	err = r.reconcileHandleService(ctx, instance)
	if err != nil {
		return robotErr.CheckCreatingOrWaitingError(result, err)
	}

	err = r.reconcileHandleIngress(ctx, instance)
	if err != nil {
		return robotErr.CheckCreatingOrWaitingError(result, err)
	}

	return nil
}

func (r *CodeEditorReconciler) reconcileCheckResources(ctx context.Context, instance *robotv1alpha2.CodeEditor) error {

	err := r.reconcileCheckPVCs(ctx, instance)
	if err != nil {
		return err
	}

	err = r.reconcileCheckExternalVolumes(ctx, instance)
	if err != nil {
		return err
	}

	err = r.reconcileCheckDeployment(ctx, instance)
	if err != nil {
		return err
	}

	err = r.reconcileCheckService(ctx, instance)
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
func (r *CodeEditorReconciler) SetupWithManager(mgr ctrl.Manager) error {
	return ctrl.NewControllerManagedBy(mgr).
		For(&robotv1alpha2.CodeEditor{}).
		Owns(&corev1.PersistentVolumeClaim{}).
		Owns(&appsv1.Deployment{}).
		Owns(&corev1.Service{}).
		Owns(&networkingv1.Ingress{}).
		Complete(r)
}
