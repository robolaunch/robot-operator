package metrics

import (
	"context"

	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	"k8s.io/apimachinery/pkg/types"
	"k8s.io/client-go/util/retry"
)

func (r *MetricsExporterReconciler) reconcileGetInstance(ctx context.Context, meta types.NamespacedName) (*robotv1alpha1.MetricsExporter, error) {
	instance := &robotv1alpha1.MetricsExporter{}
	err := r.Get(ctx, meta, instance)
	if err != nil {
		return &robotv1alpha1.MetricsExporter{}, err
	}

	return instance, nil
}

func (r *MetricsExporterReconciler) reconcileUpdateInstanceStatus(ctx context.Context, instance *robotv1alpha1.MetricsExporter) error {
	return retry.RetryOnConflict(retry.DefaultRetry, func() error {
		instanceLV := &robotv1alpha1.MetricsExporter{}
		err := r.Get(ctx, types.NamespacedName{
			Name:      instance.Name,
			Namespace: instance.Namespace,
		}, instanceLV)

		if err == nil {
			instance.ResourceVersion = instanceLV.ResourceVersion
		}

		err1 := r.Status().Update(ctx, instance)
		return err1
	})
}
