package launch_manager

import (
	"context"

	"github.com/robolaunch/robot-operator/internal"
	"github.com/robolaunch/robot-operator/internal/label"
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
		instance.Status.LaunchPodStatus.Created = true
		instance.Status.LaunchPodStatus.IP = launchPodQuery.Status.PodIP
		instance.Status.LaunchPodStatus.Phase = launchPodQuery.Status.Phase

		launchStatus := make(map[string]robotv1alpha1.LaunchStatus)
		instance.Status.LaunchPodStatus.LaunchStatus = launchStatus

		// create map, select active ones
		for k, v := range instance.Spec.Launch {
			if v.Selector == nil {
				launchStatus[k] = robotv1alpha1.LaunchStatus{
					Active: true,
				}
			} else {
				if physicalInstance, ok := v.Selector[internal.PHYSICAL_INSTANCE_LABEL_KEY]; ok {
					if physicalInstance == label.GetClusterName(robot) {
						launchStatus[k] = robotv1alpha1.LaunchStatus{
							Active: true,
						}
					} else {
						launchStatus[k] = robotv1alpha1.LaunchStatus{
							Active: false,
						}
					}
				} else if cloudInstance, ok := v.Selector[internal.CLOUD_INSTANCE_LABEL_KEY]; ok {
					if cloudInstance == label.GetClusterName(robot) {
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
