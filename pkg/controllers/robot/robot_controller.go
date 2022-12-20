package robot

import (
	"context"

	batchv1 "k8s.io/api/batch/v1"
	corev1 "k8s.io/api/core/v1"
	"k8s.io/apimachinery/pkg/api/errors"
	"k8s.io/apimachinery/pkg/runtime"
	"k8s.io/apimachinery/pkg/types"
	"k8s.io/client-go/dynamic"
	ctrl "sigs.k8s.io/controller-runtime"
	"sigs.k8s.io/controller-runtime/pkg/client"
	"sigs.k8s.io/controller-runtime/pkg/handler"
	"sigs.k8s.io/controller-runtime/pkg/log"
	"sigs.k8s.io/controller-runtime/pkg/reconcile"
	"sigs.k8s.io/controller-runtime/pkg/source"

	"github.com/go-logr/logr"
	robotv1alpha1 "github.com/robolaunch/robot-operator/api/roboscale.io/v1alpha1"
	"github.com/robolaunch/robot-operator/internal/label"
)

// RobotReconciler reconciles a Robot object
type RobotReconciler struct {
	client.Client
	Scheme        *runtime.Scheme
	DynamicClient dynamic.Interface
}

//+kubebuilder:rbac:groups=robot.roboscale.io,resources=robots,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=robot.roboscale.io,resources=robots/status,verbs=get;update;patch
//+kubebuilder:rbac:groups=robot.roboscale.io,resources=robots/finalizers,verbs=update

//+kubebuilder:rbac:groups=core,resources=nodes,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=core,resources=persistentvolumeclaims,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=robot.roboscale.io,resources=discoveryservers,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=batch,resources=jobs,verbs=get;list;watch;create;update;patch;delete

var logger logr.Logger

func (r *RobotReconciler) Reconcile(ctx context.Context, req ctrl.Request) (ctrl.Result, error) {
	logger = log.FromContext(ctx)

	instance, err := r.reconcileGetInstance(ctx, req.NamespacedName)
	if err != nil {
		if errors.IsNotFound(err) {
			return ctrl.Result{}, nil
		}
		return ctrl.Result{}, err
	}

	_, err = r.reconcileCheckNode(ctx, instance)
	if err != nil {
		return ctrl.Result{}, err
	}

	// err = r.reconcileCheckDeletion(ctx, instance)
	// if err != nil {

	// 	if errors.IsNotFound(err) {
	// 		return ctrl.Result{}, nil
	// 	}

	// 	return ctrl.Result{}, err
	// }

	err = r.reconcileCheckImage(ctx, instance)
	if err != nil {
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

func (r *RobotReconciler) reconcileCheckStatus(ctx context.Context, instance *robotv1alpha1.Robot) error {
	switch instance.Status.VolumeStatuses.Var.Created &&
		instance.Status.VolumeStatuses.Opt.Created &&
		instance.Status.VolumeStatuses.Etc.Created &&
		instance.Status.VolumeStatuses.Usr.Created &&
		instance.Status.VolumeStatuses.Workspace.Created {
	case true:

		switch instance.Status.DiscoveryServerStatus.Created {
		case true:

			switch instance.Status.DiscoveryServerStatus.Status.Phase {
			case robotv1alpha1.DiscoveryServerPhaseReady:

				switch instance.Status.LoaderJobStatus.Created {
				case true:

					switch instance.Status.LoaderJobStatus.Phase {
					case robotv1alpha1.JobSucceeded:

						switch instance.Spec.ROSBridgeTemplate.ROS.Enabled || instance.Spec.ROSBridgeTemplate.ROS2.Enabled {
						case true:

							switch instance.Status.ROSBridgeStatus.Created {
							case true:

								switch instance.Status.ROSBridgeStatus.Status.Phase {
								case robotv1alpha1.BridgePhaseReady:

									switch instance.Spec.RobotDevSuiteTemplate.IDEEnabled || instance.Spec.RobotDevSuiteTemplate.VDIEnabled {
									case true:

										switch instance.Status.RobotDevSuiteStatus.Created {
										case true:

											switch instance.Status.RobotDevSuiteStatus.Status.Phase {
											case robotv1alpha1.RobotDevSuitePhaseRunning:

												instance.Status.Phase = robotv1alpha1.RobotPhaseEnvironmentReady

												err := r.reconcileHandleAttachments(ctx, instance)
												if err != nil {
													return err
												}

											}

										case false:

											instance.Status.Phase = robotv1alpha1.RobotPhaseCreatingDevelopmentSuite
											err := r.createRobotDevSuite(ctx, instance, instance.GetRobotDevSuiteMetadata())
											if err != nil {
												return err
											}
											instance.Status.RobotDevSuiteStatus.Created = true

										}

									case false:

										instance.Status.Phase = robotv1alpha1.RobotPhaseEnvironmentReady

										err := r.reconcileHandleAttachments(ctx, instance)
										if err != nil {
											return err
										}

									}

								}

							case false:

								instance.Status.Phase = robotv1alpha1.RobotPhaseCreatingBridge
								err := r.createROSBridge(ctx, instance, instance.GetROSBridgeMetadata())
								if err != nil {
									return err
								}
								instance.Status.ROSBridgeStatus.Created = true

							}

						case false:

							switch instance.Spec.RobotDevSuiteTemplate.IDEEnabled || instance.Spec.RobotDevSuiteTemplate.VDIEnabled {
							case true:

								switch instance.Status.RobotDevSuiteStatus.Created {
								case true:

									switch instance.Status.RobotDevSuiteStatus.Status.Phase {
									case robotv1alpha1.RobotDevSuitePhaseRunning:

										instance.Status.Phase = robotv1alpha1.RobotPhaseEnvironmentReady

										err := r.reconcileHandleAttachments(ctx, instance)
										if err != nil {
											return err
										}

									}

								case false:

									instance.Status.Phase = robotv1alpha1.RobotPhaseCreatingDevelopmentSuite
									err := r.createRobotDevSuite(ctx, instance, instance.GetRobotDevSuiteMetadata())
									if err != nil {
										return err
									}
									instance.Status.RobotDevSuiteStatus.Created = true

								}

							}

						case false:

							instance.Status.Phase = robotv1alpha1.RobotPhaseEnvironmentReady

							err := r.reconcileHandleAttachments(ctx, instance)
							if err != nil {
								return err
							}

						}

					case robotv1alpha1.JobActive:

						instance.Status.Phase = robotv1alpha1.RobotPhaseConfiguringWorkspaces

					case robotv1alpha1.JobFailed:

						// TODO: add reason
						instance.Status.Phase = robotv1alpha1.RobotPhaseFailed

					}

				case false:

					instance.Status.Phase = robotv1alpha1.RobotPhaseConfiguringWorkspaces
					err := r.createJob(ctx, instance, instance.GetLoaderJobMetadata())
					if err != nil {
						return err
					}
					instance.Status.LoaderJobStatus.Created = true
				}

			}

		case false:

			instance.Status.Phase = robotv1alpha1.RobotPhaseCreatingDiscoveryServer
			err := r.createDiscoveryServer(ctx, instance, instance.GetDiscoveryServerMetadata())
			if err != nil {
				return err
			}
			instance.Status.DiscoveryServerStatus.Created = true

		}

	case false:

		instance.Status.Phase = robotv1alpha1.RobotPhaseCreatingEnvironment

		if !instance.Status.VolumeStatuses.Var.Created {
			err := r.createPVC(ctx, instance, instance.GetPVCVarMetadata())
			if err != nil {
				return err
			}
			instance.Status.VolumeStatuses.Var.Created = true
		}

		if !instance.Status.VolumeStatuses.Opt.Created {
			err := r.createPVC(ctx, instance, instance.GetPVCOptMetadata())
			if err != nil {
				return err
			}
			instance.Status.VolumeStatuses.Opt.Created = true
		}

		if !instance.Status.VolumeStatuses.Etc.Created {
			err := r.createPVC(ctx, instance, instance.GetPVCEtcMetadata())
			if err != nil {
				return err
			}
			instance.Status.VolumeStatuses.Etc.Created = true
		}

		if !instance.Status.VolumeStatuses.Usr.Created {
			err := r.createPVC(ctx, instance, instance.GetPVCUsrMetadata())
			if err != nil {
				return err
			}
			instance.Status.VolumeStatuses.Usr.Created = true
		}

		if !instance.Status.VolumeStatuses.Workspace.Created {
			err := r.createPVC(ctx, instance, instance.GetPVCWorkspaceMetadata())
			if err != nil {
				return err
			}
			instance.Status.VolumeStatuses.Workspace.Created = true
		}
	}

	return nil
}

func (r *RobotReconciler) reconcileCheckResources(ctx context.Context, instance *robotv1alpha1.Robot) error {

	err := r.reconcileCheckPVCs(ctx, instance)
	if err != nil {
		return err
	}

	err = r.reconcileCheckDiscoveryServer(ctx, instance)
	if err != nil {
		return err
	}

	err = r.reconcileCheckLoaderJob(ctx, instance)
	if err != nil {
		return err
	}

	err = r.reconcileCheckROSBridge(ctx, instance)
	if err != nil {
		return err
	}

	err = r.reconcileCheckRobotDevSuite(ctx, instance)
	if err != nil {
		return err
	}

	err = r.reconcileCheckAttachedBuildManager(ctx, instance)
	if err != nil {
		return err
	}

	err = r.reconcileCheckAttachedLaunchManager(ctx, instance)
	if err != nil {
		return err
	}

	return nil
}

// SetupWithManager sets up the controller with the Manager.
func (r *RobotReconciler) SetupWithManager(mgr ctrl.Manager) error {
	return ctrl.NewControllerManagedBy(mgr).
		For(&robotv1alpha1.Robot{}).
		Owns(&corev1.PersistentVolumeClaim{}).
		Owns(&robotv1alpha1.DiscoveryServer{}).
		Owns(&batchv1.Job{}).
		Owns(&robotv1alpha1.ROSBridge{}).
		Watches(
			&source.Kind{Type: &robotv1alpha1.BuildManager{}},
			handler.EnqueueRequestsFromMapFunc(r.watchAttachedBuildManagers),
		).
		Watches(
			&source.Kind{Type: &robotv1alpha1.LaunchManager{}},
			handler.EnqueueRequestsFromMapFunc(r.watchAttachedLaunchManagers),
		).
		Watches(
			&source.Kind{Type: &robotv1alpha1.RobotDevSuite{}},
			handler.EnqueueRequestsFromMapFunc(r.watchAttachedRobotDevSuites),
		).
		Complete(r)
}

func (r *RobotReconciler) watchAttachedBuildManagers(o client.Object) []reconcile.Request {

	obj := o.(*robotv1alpha1.BuildManager)

	robot := &robotv1alpha1.Robot{}
	err := r.Get(context.TODO(), types.NamespacedName{
		Name:      label.GetTargetRobot(obj),
		Namespace: obj.Namespace,
	}, robot)
	if err != nil {
		return []reconcile.Request{}
	}

	return []reconcile.Request{
		{
			NamespacedName: types.NamespacedName{
				Name:      robot.Name,
				Namespace: robot.Namespace,
			},
		},
	}
}

func (r *RobotReconciler) watchAttachedLaunchManagers(o client.Object) []reconcile.Request {

	obj := o.(*robotv1alpha1.LaunchManager)

	robot := &robotv1alpha1.Robot{}
	err := r.Get(context.TODO(), types.NamespacedName{
		Name:      label.GetTargetRobot(obj),
		Namespace: obj.Namespace,
	}, robot)
	if err != nil {
		return []reconcile.Request{}
	}

	return []reconcile.Request{
		{
			NamespacedName: types.NamespacedName{
				Name:      robot.Name,
				Namespace: robot.Namespace,
			},
		},
	}
}

func (r *RobotReconciler) watchAttachedRobotDevSuites(o client.Object) []reconcile.Request {

	obj := o.(*robotv1alpha1.RobotDevSuite)

	robot := &robotv1alpha1.Robot{}
	err := r.Get(context.TODO(), types.NamespacedName{
		Name:      label.GetTargetRobot(obj),
		Namespace: obj.Namespace,
	}, robot)
	if err != nil {
		return []reconcile.Request{}
	}

	return []reconcile.Request{
		{
			NamespacedName: types.NamespacedName{
				Name:      robot.Name,
				Namespace: robot.Namespace,
			},
		},
	}
}
