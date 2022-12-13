package launch_manager

import (
	"context"
	"time"

	robotv1alpha1 "github.com/robolaunch/robot-operator/api/v1alpha1"
	corev1 "k8s.io/api/core/v1"
	"k8s.io/apimachinery/pkg/api/errors"
)

func (r *LaunchManagerReconciler) reconcileDeleteLaunchPod(ctx context.Context, instance *robotv1alpha1.LaunchManager) error {

	launchPodQuery := &corev1.Pod{}
	err := r.Get(ctx, *instance.GetLaunchPodMetadata(), launchPodQuery)
	if err != nil {
		if errors.IsNotFound(err) {
			instance.Status.LaunchPodStatus = robotv1alpha1.LaunchPodStatus{}
		} else {
			return err
		}
	} else {

		err := r.Delete(ctx, launchPodQuery)
		if err != nil {
			return err
		}

		// watch until it's deleted
		deleted := false
		for !deleted {
			podQuery := &corev1.Pod{}
			err := r.Get(ctx, *instance.GetLaunchPodMetadata(), podQuery)
			if err != nil && errors.IsNotFound(err) {
				deleted = true
			}
			time.Sleep(time.Second * 1)
		}

		instance.Status.LaunchPodStatus = robotv1alpha1.LaunchPodStatus{}
	}

	return nil
}
