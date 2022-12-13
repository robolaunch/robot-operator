package launch_manager

import (
	"context"

	robotv1alpha1 "github.com/robolaunch/robot-operator/api/v1alpha1"
	corev1 "k8s.io/api/core/v1"
	"k8s.io/apimachinery/pkg/api/errors"
)

func (r *LaunchManagerReconciler) reconcileCheckLaunchPod(ctx context.Context, instance *robotv1alpha1.LaunchManager) error {

	launchPodQuery := &corev1.Pod{}
	err := r.Get(ctx, *instance.GetLaunchPodMetadata(), launchPodQuery)
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
	}

	return nil
}
