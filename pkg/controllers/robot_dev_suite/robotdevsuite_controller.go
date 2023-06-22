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

package robot_dev_suite

import (
	"context"
	goErr "errors"
	"time"

	"k8s.io/apimachinery/pkg/api/errors"
	"k8s.io/apimachinery/pkg/labels"
	"k8s.io/apimachinery/pkg/runtime"
	"k8s.io/apimachinery/pkg/selection"
	"k8s.io/apimachinery/pkg/types"
	"k8s.io/client-go/dynamic"
	ctrl "sigs.k8s.io/controller-runtime"
	"sigs.k8s.io/controller-runtime/pkg/client"
	"sigs.k8s.io/controller-runtime/pkg/handler"
	"sigs.k8s.io/controller-runtime/pkg/log"
	"sigs.k8s.io/controller-runtime/pkg/reconcile"
	"sigs.k8s.io/controller-runtime/pkg/source"

	"github.com/go-logr/logr"
	"github.com/robolaunch/robot-operator/internal"
	robotErr "github.com/robolaunch/robot-operator/internal/error"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
)

// RobotDevSuiteReconciler reconciles a RobotDevSuite object
type RobotDevSuiteReconciler struct {
	client.Client
	Scheme        *runtime.Scheme
	DynamicClient dynamic.Interface
}

//+kubebuilder:rbac:groups=robot.roboscale.io,resources=robotdevsuites,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=robot.roboscale.io,resources=robotdevsuites/status,verbs=get;update;patch
//+kubebuilder:rbac:groups=robot.roboscale.io,resources=robotdevsuites/finalizers,verbs=update

//+kubebuilder:rbac:groups=robot.roboscale.io,resources=robotides,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=robot.roboscale.io,resources=robotvdis,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=robot.roboscale.io,resources=relayservers,verbs=get;list;watch;create;update;patch;delete

var logger logr.Logger

func (r *RobotDevSuiteReconciler) Reconcile(ctx context.Context, req ctrl.Request) (ctrl.Result, error) {
	logger = log.FromContext(ctx)

	instance, err := r.reconcileGetInstance(ctx, req.NamespacedName)
	if err != nil {
		if errors.IsNotFound(err) {
			return ctrl.Result{}, nil
		}
		return ctrl.Result{}, err
	}

	// Check target robot's attached object, update activity status
	err = r.reconcileCheckTargetRobot(ctx, instance)
	if err != nil {
		if errors.IsNotFound(err) {
			instance.Status.Phase = robotv1alpha1.RobotDevSuitePhaseRobotNotFound
			instance.Status.Active = false
		} else {
			return ctrl.Result{}, err
		}
	}

	// Check target robot's other attached objects to see if robot's resources are released
	err = r.reconcileCheckOtherAttachedResources(ctx, instance)
	if err != nil {
		var e *robotErr.RobotResourcesHasNotBeenReleasedError
		if goErr.As(err, &e) {
			return ctrl.Result{
				Requeue:      true,
				RequeueAfter: 3 * time.Second,
			}, nil
		}
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

func (r *RobotDevSuiteReconciler) reconcileCheckStatus(ctx context.Context, instance *robotv1alpha1.RobotDevSuite) error {

	switch instance.Status.Active {
	case true:

		switch instance.Spec.VDIEnabled {
		case true:

			switch instance.Status.RobotVDIStatus.Resource.Created {
			case true:

				switch instance.Status.RobotVDIStatus.Resource.Phase {
				case string(robotv1alpha1.RobotVDIPhaseRunning):

					switch instance.Spec.IDEEnabled {
					case true:

						switch instance.Status.RobotIDEStatus.Resource.Created {
						case true:

							switch instance.Status.RobotIDEStatus.Resource.Phase {
							case string(robotv1alpha1.RobotIDEPhaseRunning):

								instance.Status.Phase = robotv1alpha1.RobotDevSuitePhaseRunning

							}

						case false:

							instance.Status.Phase = robotv1alpha1.RobotDevSuitePhaseCreatingRobotIDE
							err := r.reconcileCreateRobotIDE(ctx, instance)
							if err != nil {
								return err
							}
							instance.Status.RobotIDEStatus.Resource.Created = true

						}

					case false:

						instance.Status.Phase = robotv1alpha1.RobotDevSuitePhaseRunning

					}

				}

			case false:

				instance.Status.Phase = robotv1alpha1.RobotDevSuitePhaseCreatingRobotVDI
				err := r.reconcileCreateRobotVDI(ctx, instance)
				if err != nil {
					return err
				}
				instance.Status.RobotVDIStatus.Resource.Created = true

			}

		case false:

			switch instance.Spec.IDEEnabled {
			case true:

				switch instance.Status.RobotIDEStatus.Resource.Created {
				case true:

					switch instance.Status.RobotIDEStatus.Resource.Phase {
					case string(robotv1alpha1.RobotIDEPhaseRunning):

						instance.Status.Phase = robotv1alpha1.RobotDevSuitePhaseRunning

					}

				case false:

					instance.Status.Phase = robotv1alpha1.RobotDevSuitePhaseCreatingRobotIDE
					err := r.reconcileCreateRobotIDE(ctx, instance)
					if err != nil {
						return err
					}
					instance.Status.RobotIDEStatus.Resource.Created = true

				}

			case false:

				instance.Status.Phase = robotv1alpha1.RobotDevSuitePhaseRunning

			}

		}

	case false:

		instance.Status.Phase = robotv1alpha1.RobotDevSuitePhaseDeactivating

		err := r.reconcileDeleteRobotIDE(ctx, instance)
		if err != nil {
			return err
		}

		err = r.reconcileDeleteRobotVDI(ctx, instance)
		if err != nil {
			return err
		}

		instance.Status.Phase = robotv1alpha1.RobotDevSuitePhaseInactive

	}

	return nil
}

func (r *RobotDevSuiteReconciler) reconcileCheckResources(ctx context.Context, instance *robotv1alpha1.RobotDevSuite) error {

	err := r.reconcileCheckRobotVDI(ctx, instance)
	if err != nil {
		return err
	}

	err = r.reconcileCheckRobotIDE(ctx, instance)
	if err != nil {
		return err
	}

	return nil
}

// SetupWithManager sets up the controller with the Manager.
func (r *RobotDevSuiteReconciler) SetupWithManager(mgr ctrl.Manager) error {
	return ctrl.NewControllerManagedBy(mgr).
		For(&robotv1alpha1.RobotDevSuite{}).
		Owns(&robotv1alpha1.RobotVDI{}).
		Owns(&robotv1alpha1.RobotIDE{}).
		Watches(
			&source.Kind{Type: &robotv1alpha1.Robot{}},
			handler.EnqueueRequestsFromMapFunc(r.watchRobots),
		).
		Complete(r)
}

func (r *RobotDevSuiteReconciler) watchRobots(o client.Object) []reconcile.Request {

	robot := o.(*robotv1alpha1.Robot)

	// Get attached build objects for this robot
	requirements := []labels.Requirement{}
	newReq, err := labels.NewRequirement(internal.TARGET_ROBOT_LABEL_KEY, selection.In, []string{robot.Name})
	if err != nil {
		return []reconcile.Request{}
	}
	requirements = append(requirements, *newReq)

	robotSelector := labels.NewSelector().Add(requirements...)

	robotDevSuiteList := robotv1alpha1.RobotDevSuiteList{}
	err = r.List(context.TODO(), &robotDevSuiteList, &client.ListOptions{Namespace: robot.Namespace, LabelSelector: robotSelector})
	if err != nil {
		return []reconcile.Request{}
	}

	requests := make([]reconcile.Request, len(robotDevSuiteList.Items))
	for i, item := range robotDevSuiteList.Items {

		requests[i] = reconcile.Request{
			NamespacedName: types.NamespacedName{
				Name:      item.Name,
				Namespace: item.Namespace,
			},
		}

	}

	return requests
}
