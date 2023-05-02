package launch_manager

import (
	"context"

	"github.com/robolaunch/robot-operator/internal/handle"
	"github.com/robolaunch/robot-operator/internal/hybrid"
	"github.com/robolaunch/robot-operator/internal/label"
	"github.com/robolaunch/robot-operator/internal/reference"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	corev1 "k8s.io/api/core/v1"
	"k8s.io/apimachinery/pkg/api/errors"
)

func (r *LaunchManagerReconciler) reconcileCheckLaunchPod(ctx context.Context, instance *robotv1alpha1.LaunchManager) error {

	robot, err := r.reconcileGetTargetRobot(ctx, instance)
	if err != nil {
		return err
	}

	launchPodQuery := &corev1.Pod{}
	err = r.Get(ctx, *instance.GetLaunchPodMetadata(), launchPodQuery)
	if err != nil {
		if errors.IsNotFound(err) {
			instance.Status.LaunchPodStatus = robotv1alpha1.LaunchPodStatus{}
		} else {
			return err
		}
	} else {

		err := handle.HandlePod(ctx, r, *launchPodQuery)
		if err != nil {
			return err
		}

		instance.Status.LaunchPodStatus.Status.Resource.Created = true
		instance.Status.LaunchPodStatus.Status.IP = launchPodQuery.Status.PodIP
		instance.Status.LaunchPodStatus.Status.Resource.Phase = string(launchPodQuery.Status.Phase)
		reference.SetReference(&instance.Status.LaunchPodStatus.Status.Resource.Reference, launchPodQuery.TypeMeta, launchPodQuery.ObjectMeta)

		launchStatus := make(map[string]robotv1alpha1.LaunchStatus)
		instance.Status.LaunchPodStatus.LaunchStatus = launchStatus

		// create map, select active ones
		for k, v := range instance.Spec.Launch {
			if len(v.Instances) == 0 {
				launchStatus[k] = robotv1alpha1.LaunchStatus{
					Active: true,
				}
			} else {

				clusterName := label.GetClusterName(robot)

				if hybrid.ContainsInstance(v.Instances, clusterName) {
					launchStatus[k] = robotv1alpha1.LaunchStatus{
						Active: true,
					}
				} else {
					launchStatus[k] = robotv1alpha1.LaunchStatus{
						Active: false,
					}
				}
			}
		}

		// pass container statuses
		for _, containerStatus := range launchPodQuery.Status.ContainerStatuses {
			ls := launchStatus[containerStatus.Name]
			ls.ContainerStatus = containerStatus
			launchStatus[containerStatus.Name] = ls
		}

	}

	return nil
}
