package launch_manager

import (
	"context"

	"github.com/robolaunch/robot-operator/internal/label"
	resources "github.com/robolaunch/robot-operator/internal/resources/v1alpha1"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	"k8s.io/apimachinery/pkg/api/errors"
	ctrl "sigs.k8s.io/controller-runtime"
)

func (r *LaunchManagerReconciler) createLaunchPod(ctx context.Context, instance *robotv1alpha1.LaunchManager) error {

	robot, err := r.reconcileGetTargetRobot(ctx, instance)
	if err != nil {
		return err
	}

	// Use RobotVDI display if
	// - any of the launch objects needs display
	// - target VDI label is not set empty (TODO: should be deprecated)
	// - RobotVDI is created
	robotVDI := &robotv1alpha1.RobotVDI{}
	if resources.InstanceNeedDisplay(*instance, *robot) && label.GetTargetRobotVDI(instance) != "" && robot.Status.RobotDevSuiteStatus.Status.RobotVDIStatus.Resource.Created {
		robotVDI, err = r.reconcileGetTargetRobotVDI(ctx, instance, *robot)
		if err != nil {
			return err
		}
	}

	buildManager, err := r.reconcileGetCurrentBuildManager(ctx, instance)
	if err != nil {
		return err
	}

	activeNode, err := r.reconcileCheckNode(ctx, robot)
	if err != nil {
		return err
	}

	launchPod := resources.GetLaunchPod(instance, instance.GetLaunchPodMetadata(), *robot, *buildManager, *robotVDI, *activeNode)
	if err != nil {
		return err
	}

	err = ctrl.SetControllerReference(instance, launchPod, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, launchPod)
	if err != nil && errors.IsAlreadyExists(err) {
		return nil
	} else if err != nil {
		return err
	}

	logger.Info("STATUS: Launch pod is created.")
	return nil
}
