package launch_manager

import (
	"context"

	robotErr "github.com/robolaunch/robot-operator/internal/error"
	"github.com/robolaunch/robot-operator/internal/hybrid"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	corev1 "k8s.io/api/core/v1"
)

func (r *LaunchManagerReconciler) reconcileHandlePod(ctx context.Context, instance *robotv1alpha1.LaunchManager) error {

	robot, err := r.reconcileGetTargetRobot(ctx, instance)
	if err != nil {
		return err
	}

	if hybrid.HasLaunchInThisInstance(*instance, *robot) {

		if !instance.Status.LaunchPodStatus.Status.Resource.Created {
			instance.Status.Phase = robotv1alpha1.LaunchManagerPhaseCreatingPod
			err := r.createLaunchPod(ctx, instance)
			if err != nil {
				return err
			}
			instance.Status.LaunchPodStatus.Status.Resource.Created = true
			instance.Status.Phase = robotv1alpha1.LaunchManagerPhaseLaunching

			return &robotErr.CreatingResourceError{
				ResourceKind:      "Pod",
				ResourceName:      instance.GetLaunchPodMetadata().Name,
				ResourceNamespace: instance.GetLaunchPodMetadata().Namespace,
			}
		}

		if instance.Status.LaunchPodStatus.Status.Resource.Phase != string(corev1.PodRunning) {
			return &robotErr.WaitingForResourceError{
				ResourceKind:      "Pod",
				ResourceName:      instance.GetLaunchPodMetadata().Name,
				ResourceNamespace: instance.GetLaunchPodMetadata().Namespace,
			}
		}

	}

	return nil
}
