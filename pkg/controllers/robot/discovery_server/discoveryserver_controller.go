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

package discovery_server

import (
	"context"
	goErr "errors"
	"time"

	corev1 "k8s.io/api/core/v1"
	"k8s.io/apimachinery/pkg/api/errors"
	"k8s.io/apimachinery/pkg/runtime"
	"k8s.io/client-go/dynamic"
	ctrl "sigs.k8s.io/controller-runtime"
	"sigs.k8s.io/controller-runtime/pkg/client"
	"sigs.k8s.io/controller-runtime/pkg/log"

	"github.com/go-logr/logr"
	robotErr "github.com/robolaunch/robot-operator/internal/error"
	mcsv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/external/apis/mcsv1alpha1/v1alpha1"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
)

// DiscoveryServerReconciler reconciles a DiscoveryServer object
type DiscoveryServerReconciler struct {
	client.Client
	Scheme        *runtime.Scheme
	DynamicClient dynamic.Interface
}

//+kubebuilder:rbac:groups=robot.roboscale.io,resources=discoveryservers,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=robot.roboscale.io,resources=discoveryservers/status,verbs=get;update;patch
//+kubebuilder:rbac:groups=robot.roboscale.io,resources=discoveryservers/finalizers,verbs=update

//+kubebuilder:rbac:groups=core,resources=pods,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=core,resources=services,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=core,resources=configmaps,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=multicluster.x-k8s.io,resources=serviceexports,verbs=get;list;watch;create;update;patch;delete

var logger logr.Logger

func (r *DiscoveryServerReconciler) Reconcile(ctx context.Context, req ctrl.Request) (ctrl.Result, error) {
	logger = log.FromContext(ctx)

	instance, err := r.reconcileGetInstance(ctx, req.NamespacedName)
	if err != nil {
		if errors.IsNotFound(err) {
			return ctrl.Result{}, nil
		}
		return ctrl.Result{}, err
	}

	// err = r.reconcileCheckDeletion(ctx, instance)
	// if err != nil {

	// 	if errors.IsNotFound(err) {
	// 		return ctrl.Result{}, nil
	// 	}

	// 	return ctrl.Result{}, err
	// }

	err = r.reconcileCheckStatus(ctx, instance)
	if err != nil {
		return ctrl.Result{}, err
	}

	err = r.reconcileUpdateInstanceStatus(ctx, instance)
	if err != nil {
		return ctrl.Result{}, err
	}

	err = r.reconcileUpdateConnectionInfo(ctx, instance)
	if err != nil {
		var e *robotErr.CannotResolveDiscoveryServerError
		if goErr.As(err, &e) {
			logger.Info("STATUS: Trying to resolve discovery server DNS.")
			return ctrl.Result{
				Requeue:      true,
				RequeueAfter: 3 * time.Second,
			}, nil
		}
		return ctrl.Result{}, nil
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

func (r *DiscoveryServerReconciler) reconcileCheckStatus(ctx context.Context, instance *robotv1alpha1.DiscoveryServer) error {

	switch instance.Spec.Type {
	case robotv1alpha1.DiscoveryServerInstanceTypeServer:

		switch instance.Status.ServiceStatus.Created {
		case true:

			switch instance.Status.PodStatus.Created {
			case true:

				switch instance.Status.PodStatus.Phase {
				case corev1.PodRunning:

					// TODO: Cover other pod phases

					switch instance.Status.ServiceExportStatus.Created {
					case true:

						switch instance.Status.ConfigMapStatus.Created {
						case true:

							instance.Status.Phase = robotv1alpha1.DiscoveryServerPhaseReady

						case false:

							instance.Status.Phase = robotv1alpha1.DiscoveryServerPhaseCreatingConfigMap

							if instance.Status.ConnectionInfo.IP != "" {

								err := r.createConfigMap(ctx, instance, instance.GetDiscoveryServerConfigMapMetadata())
								if err != nil {
									return err
								}
								instance.Status.ConfigMapStatus.Created = true

							}
						}

					case false:

						instance.Status.Phase = robotv1alpha1.DiscoveryServerPhaseCreatingServiceExport
						err := r.createServiceExport(ctx, instance, instance.GetDiscoveryServerServiceMetadata())
						if err != nil {
							return err
						}
						instance.Status.ServiceExportStatus.Created = true

					}

				}

			case false:

				instance.Status.Phase = robotv1alpha1.DiscoveryServerPhaseCreatingPod
				err := r.createPod(ctx, instance, instance.GetDiscoveryServerPodMetadata())
				if err != nil {
					return err
				}
				instance.Status.PodStatus.Created = true

			}

		case false:

			instance.Status.Phase = robotv1alpha1.DiscoveryServerPhaseCreatingService
			err := r.createService(ctx, instance, instance.GetDiscoveryServerPodMetadata())
			if err != nil {
				return err
			}
			instance.Status.ServiceStatus.Created = true

		}

	case robotv1alpha1.DiscoveryServerInstanceTypeClient:

		if instance.Status.ConnectionInfo.IP != "" {

			switch instance.Status.ConfigMapStatus.Created {
			case true:

				instance.Status.Phase = robotv1alpha1.DiscoveryServerPhaseReady

			case false:

				instance.Status.Phase = robotv1alpha1.DiscoveryServerPhaseCreatingConfigMap

				err := r.createConfigMap(ctx, instance, instance.GetDiscoveryServerConfigMapMetadata())
				if err != nil {
					return err
				}
				instance.Status.ConfigMapStatus.Created = true

			}

		}

	}

	return nil
}

func (r *DiscoveryServerReconciler) reconcileCheckResources(ctx context.Context, instance *robotv1alpha1.DiscoveryServer) error {

	if instance.Spec.Type == robotv1alpha1.DiscoveryServerInstanceTypeServer {
		err := r.reconcileCheckService(ctx, instance)
		if err != nil {
			return err
		}

		err = r.reconcileCheckPod(ctx, instance)
		if err != nil {
			return err
		}
	}

	err := r.reconcileCheckConfigMap(ctx, instance)
	if err != nil {
		return err
	}

	return nil
}

// SetupWithManager sets up the controller with the Manager.
func (r *DiscoveryServerReconciler) SetupWithManager(mgr ctrl.Manager) error {
	return ctrl.NewControllerManagedBy(mgr).
		For(&robotv1alpha1.DiscoveryServer{}).
		Owns(&corev1.Pod{}).
		Owns(&corev1.Service{}).
		Owns(&corev1.ConfigMap{}).
		Owns(&mcsv1alpha1.ServiceExport{}).
		Complete(r)
}
