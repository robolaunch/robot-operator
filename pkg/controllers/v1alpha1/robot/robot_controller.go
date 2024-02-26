package robot

import (
	"context"

	batchv1 "k8s.io/api/batch/v1"
	corev1 "k8s.io/api/core/v1"
	"k8s.io/apimachinery/pkg/api/errors"
	"k8s.io/apimachinery/pkg/runtime"
	"k8s.io/apimachinery/pkg/types"
	"k8s.io/client-go/dynamic"
	"k8s.io/client-go/kubernetes"
	ctrl "sigs.k8s.io/controller-runtime"
	"sigs.k8s.io/controller-runtime/pkg/client"
	"sigs.k8s.io/controller-runtime/pkg/handler"
	"sigs.k8s.io/controller-runtime/pkg/log"
	"sigs.k8s.io/controller-runtime/pkg/reconcile"
	"sigs.k8s.io/controller-runtime/pkg/source"

	"github.com/go-logr/logr"
	robotErr "github.com/robolaunch/robot-operator/internal/error"
	"github.com/robolaunch/robot-operator/internal/label"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
)

// RobotReconciler reconciles a Robot object
type RobotReconciler struct {
	client.Client
	Scheme        *runtime.Scheme
	DynamicClient dynamic.Interface
	Clientset     kubernetes.Clientset
}

//+kubebuilder:rbac:groups=robot.roboscale.io,resources=robots,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=robot.roboscale.io,resources=robots/status,verbs=get;update;patch
//+kubebuilder:rbac:groups=robot.roboscale.io,resources=robots/finalizers,verbs=update

//+kubebuilder:rbac:groups=core,resources=nodes,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=core,resources=persistentvolumeclaims,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=core,resources=pods/log,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=robot.roboscale.io,resources=discoveryservers,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=batch,resources=jobs,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=robot.roboscale.io,resources=rosbridges,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=robot.roboscale.io,resources=workspacemanagers,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=robot.roboscale.io,resources=buildmanagers,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=robot.roboscale.io,resources=launchmanagers,verbs=get;list;watch;create;update;patch;delete

var logger logr.Logger

func (r *RobotReconciler) Reconcile(ctx context.Context, req ctrl.Request) (ctrl.Result, error) {
	logger = log.FromContext(ctx)

	var result ctrl.Result = ctrl.Result{}

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

	err = r.reconcileCheckImage(ctx, instance)
	if err != nil {
		return ctrl.Result{}, err
	}

	err = r.reconcileCheckPersistentDirectories(ctx, instance)
	if err != nil {
		return ctrl.Result{}, err
	}

	err = r.reconcileCheckHostDirectories(ctx, instance)
	if err != nil {
		return ctrl.Result{}, err
	}

	err = r.reconcileCheckStatus(ctx, instance, &result)
	if err != nil {
		return result, err
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

	return result, nil
}

func (r *RobotReconciler) reconcileCheckStatus(ctx context.Context, instance *robotv1alpha1.Robot, result *ctrl.Result) error {

	if instance.Spec.Type == robotv1alpha1.TypeRobot {

		err := r.reconcileCheckStatusForRobot(ctx, instance, result)
		if err != nil {
			return err
		}

	} else if instance.Spec.Type == robotv1alpha1.TypeEnvironment {
		err := r.reconcileCheckStatusForEnvironment(ctx, instance, result)
		if err != nil {
			return err
		}
	}

	return nil
}

func (r *RobotReconciler) reconcileCheckStatusForRobot(ctx context.Context, instance *robotv1alpha1.Robot, result *ctrl.Result) error {

	err := r.reconcileHandlePVCs(ctx, instance)
	if err != nil {
		return robotErr.CheckCreatingOrWaitingError(result, err)
	}

	err = r.reconcileHandleDiscoveryServer(ctx, instance)
	if err != nil {
		return robotErr.CheckCreatingOrWaitingError(result, err)
	}

	err = r.reconcileHandleLoaderJob(ctx, instance)
	if err != nil {
		return robotErr.CheckCreatingOrWaitingError(result, err)
	}

	err = r.reconcileHandleROSBridge(ctx, instance)
	if err != nil {
		return robotErr.CheckCreatingOrWaitingError(result, err)
	}

	err = r.reconcileHandleRobotDevSuite(ctx, instance)
	if err != nil {
		return robotErr.CheckCreatingOrWaitingError(result, err)
	}

	err = r.reconcileHandleWorkspaceManager(ctx, instance)
	if err != nil {
		return robotErr.CheckCreatingOrWaitingError(result, err)
	}

	err = r.reconcileHandleManagers(ctx, instance)
	if err != nil {
		return robotErr.CheckCreatingOrWaitingError(result, err)
	}

	return nil
}

func (r *RobotReconciler) reconcileCheckStatusForEnvironment(ctx context.Context, instance *robotv1alpha1.Robot, result *ctrl.Result) error {

	err := r.reconcileHandlePVCs(ctx, instance)
	if err != nil {
		return robotErr.CheckCreatingOrWaitingError(result, err)
	}

	err = r.reconcileHandleLoaderJob(ctx, instance)
	if err != nil {
		return robotErr.CheckCreatingOrWaitingError(result, err)
	}

	err = r.reconcileHandleRobotDevSuite(ctx, instance)
	if err != nil {
		return robotErr.CheckCreatingOrWaitingError(result, err)
	}

	err = r.reconcileHandleWorkspaceManager(ctx, instance)
	if err != nil {
		return robotErr.CheckCreatingOrWaitingError(result, err)
	}

	err = r.reconcileHandleManagers(ctx, instance)
	if err != nil {
		return robotErr.CheckCreatingOrWaitingError(result, err)
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

	err = r.reconcileCheckWorkspaceManager(ctx, instance)
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
		Owns(&robotv1alpha1.WorkspaceManager{}).
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
