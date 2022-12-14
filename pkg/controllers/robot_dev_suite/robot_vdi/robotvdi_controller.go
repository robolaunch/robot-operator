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

package robot_vdi

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
	robotv1alpha1 "github.com/robolaunch/robot-operator/api/v1alpha1"
)

// RobotVDIReconciler reconciles a RobotVDI object
type RobotVDIReconciler struct {
	client.Client
	Scheme        *runtime.Scheme
	DynamicClient dynamic.Interface
}

//+kubebuilder:rbac:groups=robot.roboscale.io,resources=robotvdis,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=robot.roboscale.io,resources=robotvdis/status,verbs=get;update;patch
//+kubebuilder:rbac:groups=robot.roboscale.io,resources=robotvdis/finalizers,verbs=update

var logger logr.Logger

func (r *RobotVDIReconciler) Reconcile(ctx context.Context, req ctrl.Request) (ctrl.Result, error) {
	logger = log.FromContext(ctx)

	instance, err := r.reconcileGetInstance(ctx, req.NamespacedName)
	if err != nil {
		if errors.IsNotFound(err) {
			return ctrl.Result{}, nil
		}
		return ctrl.Result{}, err
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

func (r *RobotVDIReconciler) reconcileCheckStatus(ctx context.Context, instance *robotv1alpha1.RobotVDI) error {

	switch instance.Status.PVCStatus.Created || instance.Spec.MainDisplay {
	case true:

		switch instance.Status.ServiceTCPStatus.Created {
		case true:

			switch instance.Status.ServiceUDPStatus.Created {
			case true:

				switch instance.Status.PodStatus.Created {
				case true:

					switch instance.Status.PodStatus.Phase {
					case corev1.PodRunning:

						switch instance.Spec.Ingress {
						case true:

							switch instance.Status.IngressStatus.Created {
							case true:

								// ready

							case false:

								instance.Status.Phase = robotv1alpha1.RobotVDIPhaseCreatingIngress
								// create ingress

							}

						case false:
							// ready
						}

					}

				case false:

					instance.Status.Phase = robotv1alpha1.RobotVDIPhaseCreatingPod
					// create pod

				}

			case false:

				instance.Status.Phase = robotv1alpha1.RobotVDIPhaseCreatingUDPService
				// create service

			}

		case false:

			instance.Status.Phase = robotv1alpha1.RobotVDIPhaseCreatingPVC
			// create service

		}

	case false:

		// create pvc

	}

	return nil
}

func (r *RobotVDIReconciler) reconcileCheckResources(ctx context.Context, instance *robotv1alpha1.RobotVDI) error {

	err := r.reconcileCheckPVC(ctx, instance)
	if err != nil {
		return err
	}

	err = r.reconcileCheckServices(ctx, instance)
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
func (r *RobotVDIReconciler) SetupWithManager(mgr ctrl.Manager) error {
	return ctrl.NewControllerManagedBy(mgr).
		For(&robotv1alpha1.RobotVDI{}).
		Owns(&corev1.Service{}).
		Owns(&corev1.Pod{}).
		Complete(r)
}
