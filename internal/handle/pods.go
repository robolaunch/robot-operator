package handle

import (
	"context"

	corev1 "k8s.io/api/core/v1"
	"sigs.k8s.io/controller-runtime/pkg/client"
)

func HandlePod(ctx context.Context, writer client.Writer, pod corev1.Pod) error {

	if pod.Status.Phase == corev1.PodUnknown || pod.Status.Phase == corev1.PodSucceeded || pod.Status.Phase == corev1.PodFailed {
		err := writer.Delete(ctx, &pod)
		if err != nil {
			return err
		}
	}

	return nil
}
