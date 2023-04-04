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
	ctrl "sigs.k8s.io/controller-runtime"
	"sigs.k8s.io/controller-runtime/pkg/client"
	"sigs.k8s.io/controller-runtime/pkg/log"

	"github.com/go-logr/logr"
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

//+kubebuilder:rbac:groups=core,resources=pods,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=core,resources=serviceaccounts,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=rbac.authorization.k8s.io,resources=roles,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=rbac.authorization.k8s.io,resources=rolebindings,verbs=get;list;watch;create;update;patch;delete

var logger logr.Logger

func (r *MetricsExporterReconciler) Reconcile(ctx context.Context, req ctrl.Request) (ctrl.Result, error) {
	logger = log.FromContext(ctx)

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

func (r *MetricsExporterReconciler) reconcileCheckStatus(ctx context.Context, instance *robotv1alpha1.MetricsExporter) error {

	if instance.Spec.GPU.Track || instance.Spec.Network.Track {

		switch instance.Status.RoleStatus.Created {
		case true:

			switch instance.Status.ServiceAccountStatus.Created {
			case true:

				switch instance.Status.RoleBindingStatus.Created {
				case true:

					switch instance.Status.PodStatus.Created {
					case true:

						switch instance.Status.PodStatus.Phase {
						case string(corev1.PodRunning):

							instance.Status.Phase = robotv1alpha1.MetricsExporterPhaseReady

						}

					case false:

						instance.Status.Phase = robotv1alpha1.MetricsExporterPhaseCreatingPod
						err := r.reconcileCreatePod(ctx, instance)
						if err != nil {
							return err
						}
						instance.Status.PodStatus.Created = true

					}

				case false:

					instance.Status.Phase = robotv1alpha1.MetricsExporterPhaseCreatingRoleBinding
					err := r.reconcileCreateRoleBinding(ctx, instance)
					if err != nil {
						return err
					}
					instance.Status.RoleBindingStatus.Created = true

				}

			case false:

				instance.Status.Phase = robotv1alpha1.MetricsExporterPhaseCreatingServiceAccount
				err := r.reconcileCreateServiceAccount(ctx, instance)
				if err != nil {
					return err
				}
				instance.Status.ServiceAccountStatus.Created = true

			}

		case false:

			instance.Status.Phase = robotv1alpha1.MetricsExporterPhaseCreatingRole
			err := r.reconcileCreateRole(ctx, instance)
			if err != nil {
				return err
			}
			instance.Status.RoleStatus.Created = true

		}

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
		Complete(r)
}
