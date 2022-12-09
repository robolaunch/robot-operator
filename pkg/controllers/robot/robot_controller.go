package robot

import (
	"context"

	batchv1 "k8s.io/api/batch/v1"
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
	switch instance.Status.VolumeStatus.Var &&
		instance.Status.VolumeStatus.Opt &&
		instance.Status.VolumeStatus.Etc &&
		instance.Status.VolumeStatus.Usr &&
		instance.Status.VolumeStatus.Display &&
		instance.Status.VolumeStatus.Workspace {
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

								instance.Status.Phase = robotv1alpha1.RobotPhaseReady

							case false:

								instance.Status.Phase = robotv1alpha1.RobotPhaseCreatingBridge
								err := r.createROSBridge(ctx, instance, instance.GetROSBridgeMetadata())
								if err != nil {
									return err
								}
								instance.Status.ROSBridgeStatus.Created = true

							}

						case false:

							instance.Status.Phase = robotv1alpha1.RobotPhaseReady

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

			// create discovery server

			instance.Status.Phase = robotv1alpha1.RobotPhaseCreatingDiscoveryServer
			err := r.createDiscoveryServer(ctx, instance, instance.GetDiscoveryServerMetadata())
			if err != nil {
				return err
			}
			instance.Status.DiscoveryServerStatus.Created = true

		}

	case false:

		instance.Status.Phase = robotv1alpha1.RobotPhaseCreatingEnvironment

		if !instance.Status.VolumeStatus.Var {
			err := r.createPVC(ctx, instance, instance.GetPVCVarMetadata())
			if err != nil {
				return err
			}
			instance.Status.VolumeStatus.Var = true
		}

		if !instance.Status.VolumeStatus.Opt {
			err := r.createPVC(ctx, instance, instance.GetPVCOptMetadata())
			if err != nil {
				return err
			}
			instance.Status.VolumeStatus.Opt = true
		}

		if !instance.Status.VolumeStatus.Etc {
			err := r.createPVC(ctx, instance, instance.GetPVCEtcMetadata())
			if err != nil {
				return err
			}
			instance.Status.VolumeStatus.Etc = true
		}

		if !instance.Status.VolumeStatus.Usr {
			err := r.createPVC(ctx, instance, instance.GetPVCUsrMetadata())
			if err != nil {
				return err
			}
			instance.Status.VolumeStatus.Usr = true
		}

		if !instance.Status.VolumeStatus.Display {
			err := r.createPVC(ctx, instance, instance.GetPVCDisplayMetadata())
			if err != nil {
				return err
			}
			instance.Status.VolumeStatus.Display = true
		}

		if !instance.Status.VolumeStatus.Workspace {
			err := r.createPVC(ctx, instance, instance.GetPVCWorkspaceMetadata())
			if err != nil {
				return err
			}
			instance.Status.VolumeStatus.Workspace = true
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

	return nil
}

// SetupWithManager sets up the controller with the Manager.
func (r *RobotReconciler) SetupWithManager(mgr ctrl.Manager) error {
	return ctrl.NewControllerManagedBy(mgr).
		For(&robotv1alpha1.Robot{}).
		Owns(&corev1.PersistentVolumeClaim{}).
		Owns(&robotv1alpha1.DiscoveryServer{}).
		Owns(&batchv1.Job{}).
		Complete(r)
}
