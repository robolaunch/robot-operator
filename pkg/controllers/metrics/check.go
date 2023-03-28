package metrics

import (
	"context"

	"github.com/robolaunch/robot-operator/internal/handle"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	corev1 "k8s.io/api/core/v1"
	"k8s.io/apimachinery/pkg/api/errors"
)

func (r *MetricsExporterReconciler) reconcileCheckPod(ctx context.Context, instance *robotv1alpha1.MetricsExporter) error {

	podQuery := &corev1.Pod{}
	err := r.Get(ctx, *instance.GetMetricsExporterPodMetadata(), podQuery)
	if err != nil {
		if errors.IsNotFound(err) {
			instance.Status.PodStatus = robotv1alpha1.MetricsExporterPodStatus{}
		} else {
			return err
		}
	} else {

		err = handle.HandlePod(ctx, r, *podQuery)
		if err != nil {
			return err
		}

		instance.Status.PodStatus.Created = true
		instance.Status.PodStatus.Phase = podQuery.Status.Phase
	}

	return nil
}
