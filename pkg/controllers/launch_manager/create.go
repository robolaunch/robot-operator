package launch_manager

import (
	"context"

	robotv1alpha1 "github.com/robolaunch/robot-operator/api/v1alpha1"
	"github.com/robolaunch/robot-operator/internal/resources"
	ctrl "sigs.k8s.io/controller-runtime"
)

func (r *LaunchManagerReconciler) createLaunchPod(ctx context.Context, instance *robotv1alpha1.LaunchManager) error {

	robot, err := r.reconcileGetTargetRobot(ctx, instance)
	if err != nil {
		return err
	}

	robotVDI, err := r.reconcileGetTargetRobotVDI(ctx, instance)
	if err != nil {
		return err
	}

	buildManager, err := r.reconcileGetCurrentBuildManager(ctx, instance)
	if err != nil {
		return err
	}

	launchPod := resources.GetLaunchPod(instance, instance.GetLaunchPodMetadata(), *robot, *buildManager, *robotVDI)
	if err != nil {
		return err
	}

	err = ctrl.SetControllerReference(instance, launchPod, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, launchPod)
	if err != nil {
		return err
	}

	logger.Info("STATUS: Launch pod is created.")
	return nil
}
