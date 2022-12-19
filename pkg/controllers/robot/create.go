package robot

import (
	"context"
	"reflect"
	"strconv"

	robotv1alpha1 "github.com/robolaunch/robot-operator/api/roboscale.io/v1alpha1"
	"github.com/robolaunch/robot-operator/internal/node"
	"github.com/robolaunch/robot-operator/internal/resources"
	"k8s.io/apimachinery/pkg/api/errors"
	"k8s.io/apimachinery/pkg/types"
	ctrl "sigs.k8s.io/controller-runtime"
)

func (r *RobotReconciler) createPVC(ctx context.Context, instance *robotv1alpha1.Robot, pvcNamespacedName *types.NamespacedName) error {

	pvc := resources.GetPersistentVolumeClaim(instance, pvcNamespacedName)

	err := ctrl.SetControllerReference(instance, pvc, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, pvc)
	if err != nil && errors.IsAlreadyExists(err) {
		return nil
	} else if err != nil {
		return err
	}

	logger.Info("STATUS: PVC " + pvc.Name + " is created.")
	return nil
}

func (r *RobotReconciler) createDiscoveryServer(ctx context.Context, instance *robotv1alpha1.Robot, dsNamespacedName *types.NamespacedName) error {

	discoveryServer := resources.GetDiscoveryServer(instance, dsNamespacedName)

	err := ctrl.SetControllerReference(instance, discoveryServer, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, discoveryServer)
	if err != nil && errors.IsAlreadyExists(err) {
		return nil
	} else if err != nil {
		return err
	}

	logger.Info("STATUS: Discovery server " + discoveryServer.Name + " is created.")
	return nil
}

func (r *RobotReconciler) createJob(ctx context.Context, instance *robotv1alpha1.Robot, jobNamespacedName *types.NamespacedName) error {

	activeNode, err := r.reconcileCheckNode(ctx, instance)
	if err != nil {
		return err
	}

	job := resources.GetLoaderJob(instance, jobNamespacedName, node.HasGPU(*activeNode))

	err = ctrl.SetControllerReference(instance, job, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, job)
	if err != nil && errors.IsAlreadyExists(err) {
		return nil
	} else if err != nil {
		return err
	}

	logger.Info("STATUS: Job " + job.Name + " is created.")
	return nil
}

func (r *RobotReconciler) createROSBridge(ctx context.Context, instance *robotv1alpha1.Robot, bridgeNamespacedName *types.NamespacedName) error {

	rosBridge := resources.GetROSBridge(instance, bridgeNamespacedName)

	err := ctrl.SetControllerReference(instance, rosBridge, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, rosBridge)
	if err != nil && errors.IsAlreadyExists(err) {
		return nil
	} else if err != nil {
		return err
	}

	logger.Info("STATUS: ROS bridge " + rosBridge.Name + " is created.")
	return nil
}

func (r *RobotReconciler) createRobotDevSuite(ctx context.Context, instance *robotv1alpha1.Robot, rdsNamespacedName *types.NamespacedName) error {

	robotDevSuite := resources.GetRobotDevSuite(instance, rdsNamespacedName)

	err := ctrl.SetControllerReference(instance, robotDevSuite, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, robotDevSuite)
	if err != nil && errors.IsAlreadyExists(err) {
		return nil
	} else if err != nil {
		return err
	}

	logger.Info("STATUS: Robot dev suite " + robotDevSuite.Name + " is created.")
	return nil
}

func (r *RobotReconciler) createBuildManager(ctx context.Context, instance *robotv1alpha1.Robot) error {

	if reflect.DeepEqual(instance.Status.InitialBuildManagerStatus, robotv1alpha1.ManagerStatus{}) && !reflect.DeepEqual(instance.Spec.BuildManagerTemplate, robotv1alpha1.BuildManagerSpec{}) {
		buildManager := resources.GetBuildManager(instance, &types.NamespacedName{Namespace: instance.Namespace, Name: instance.Name + "-build"})

		err := ctrl.SetControllerReference(instance, buildManager, r.Scheme)
		if err != nil {
			return err
		}

		err = r.Create(ctx, buildManager)
		if err != nil && errors.IsAlreadyExists(err) {
			return nil
		} else if err != nil {
			return err
		}

		logger.Info("STATUS: Build manager " + buildManager.Name + " is created.")

		instance.Status.InitialBuildManagerStatus.Created = true
		instance.Status.InitialBuildManagerStatus.Name = instance.Name + "-build"
	}

	return nil
}

func (r *RobotReconciler) createLaunchManager(ctx context.Context, instance *robotv1alpha1.Robot, lmNamespacedName *types.NamespacedName, key int) error {

	launchManager := resources.GetLaunchManager(instance, lmNamespacedName, key)

	err := ctrl.SetControllerReference(instance, launchManager, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, launchManager)
	if err != nil && errors.IsAlreadyExists(err) {
		return nil
	} else if err != nil {
		return err
	}

	logger.Info("STATUS: Launch manager " + launchManager.Name + " is created.")
	return nil
}

func (r *RobotReconciler) createLaunchManagers(ctx context.Context, instance *robotv1alpha1.Robot) error {

	if len(instance.Status.InitialLaunchManagerStatuses) == 0 {
		for key := range instance.Spec.LaunchManagerTemplates {
			instance.Status.InitialLaunchManagerStatuses = append(instance.Status.InitialLaunchManagerStatuses, robotv1alpha1.ManagerStatus{
				Name: instance.Name + "-launch-" + strconv.Itoa(key),
			})
		}

		for key, lm := range instance.Status.InitialLaunchManagerStatuses {
			if !lm.Created {
				err := r.createLaunchManager(ctx, instance, &types.NamespacedName{Namespace: instance.Namespace, Name: lm.Name}, key)
				if err != nil {
					return err
				}
				lm.Created = true
				instance.Status.InitialLaunchManagerStatuses[key] = lm
			}
		}
	}

	return nil
}
