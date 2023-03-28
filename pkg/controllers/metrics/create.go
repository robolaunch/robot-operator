package metrics

import (
	"context"

	"github.com/robolaunch/robot-operator/internal/resources"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	corev1 "k8s.io/api/core/v1"
	"k8s.io/apimachinery/pkg/api/errors"
	ctrl "sigs.k8s.io/controller-runtime"
)

func (r *MetricsExporterReconciler) reconcileCreatePod(ctx context.Context, instance *robotv1alpha1.MetricsExporter) error {

	metricsPod := resources.GetMetricsExporterPod(instance, instance.GetMetricsExporterPodMetadata(), corev1.Node{})

	err := ctrl.SetControllerReference(instance, metricsPod, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, metricsPod)
	if err != nil && errors.IsAlreadyExists(err) {
		return nil
	} else if err != nil {
		return err
	}

	logger.Info("STATUS: Metrics pod is created.")

	return nil
}
