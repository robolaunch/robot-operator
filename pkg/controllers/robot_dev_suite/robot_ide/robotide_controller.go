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

package robot_ide

import (
	"context"

	corev1 "k8s.io/api/core/v1"
	networkingv1 "k8s.io/api/networking/v1"
	"k8s.io/apimachinery/pkg/api/errors"
	"k8s.io/apimachinery/pkg/runtime"
	"k8s.io/client-go/dynamic"
	ctrl "sigs.k8s.io/controller-runtime"
	"sigs.k8s.io/controller-runtime/pkg/client"
	"sigs.k8s.io/controller-runtime/pkg/log"

	"github.com/go-logr/logr"
	"github.com/robolaunch/robot-operator/internal/label"
	mcsv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/external/apis/mcsv1alpha1/v1alpha1"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
)

// RobotIDEReconciler reconciles a RobotIDE object
type RobotIDEReconciler struct {
	client.Client
	Scheme        *runtime.Scheme
	DynamicClient dynamic.Interface
}

//+kubebuilder:rbac:groups=robot.roboscale.io,resources=robotides,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=robot.roboscale.io,resources=robotides/status,verbs=get;update;patch
//+kubebuilder:rbac:groups=robot.roboscale.io,resources=robotides/finalizers,verbs=update

//+kubebuilder:rbac:groups=core,resources=pods,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=core,resources=services,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=networking.k8s.io,resources=ingresses,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=multicluster.x-k8s.io,resources=serviceexports,verbs=get;list;watch;create;update;patch;delete

var logger logr.Logger

func (r *RobotIDEReconciler) Reconcile(ctx context.Context, req ctrl.Request) (ctrl.Result, error) {
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

func (r *RobotIDEReconciler) reconcileCheckStatus(ctx context.Context, instance *robotv1alpha1.RobotIDE) error {

	switch instance.Status.ServiceStatus.Resource.Created {
	case true:

		switch instance.Status.PodStatus.Resource.Created {
		case true:

			switch instance.Status.IngressStatus.Created || !instance.Spec.Ingress {
			case true:

				switch instance.Status.PodStatus.Resource.Phase {
				case string(corev1.PodRunning):

					instance.Status.Phase = robotv1alpha1.RobotIDEPhaseRunning

				}

			case false:

				instance.Status.Phase = robotv1alpha1.RobotIDEPhaseCreatingIngress
				err := r.reconcileCreateIngress(ctx, instance)
				if err != nil {
					return err
				}
				instance.Status.IngressStatus.Created = true

			}

		case false:

			instance.Status.Phase = robotv1alpha1.RobotIDEPhaseCreatingPod
			err := r.reconcileCreatePod(ctx, instance)
			if err != nil {
				return err
			}
			instance.Status.PodStatus.Resource.Created = true
		}

	case false:

		instance.Status.Phase = robotv1alpha1.RobotIDEPhaseCreatingService
		err := r.reconcileCreateService(ctx, instance)
		if err != nil {
			return err
		}
		instance.Status.ServiceStatus.Resource.Created = true

	}

	switch label.GetInstanceType(instance) == label.InstanceTypePhysicalInstance && !instance.Status.ServiceExportStatus.Created {
	case true:
		err := r.reconcileCreateServiceExport(ctx, instance)
		if err != nil {
			return err
		}
		instance.Status.ServiceExportStatus.Created = true
	}

	return nil
}

func (r *RobotIDEReconciler) reconcileCheckResources(ctx context.Context, instance *robotv1alpha1.RobotIDE) error {

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

	if label.GetInstanceType(instance) == label.InstanceTypePhysicalInstance {
		err = r.reconcileCheckServiceExport(ctx, instance)
		if err != nil {
			return err
		}
	}

	return nil
}

// SetupWithManager sets up the controller with the Manager.
func (r *RobotIDEReconciler) SetupWithManager(mgr ctrl.Manager) error {
	return ctrl.NewControllerManagedBy(mgr).
		For(&robotv1alpha1.RobotIDE{}).
		Owns(&corev1.Pod{}).
		Owns(&corev1.Service{}).
		Owns(&networkingv1.Ingress{}).
		Owns(&mcsv1alpha1.ServiceExport{}).
		Complete(r)
}
