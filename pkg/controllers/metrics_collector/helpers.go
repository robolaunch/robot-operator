package metrics_collector

import (
	"context"

	"github.com/robolaunch/robot-operator/internal/label"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	"k8s.io/apimachinery/pkg/types"
	"k8s.io/client-go/util/retry"
)

func (r *MetricsCollectorReconciler) reconcileGetInstance(ctx context.Context, meta types.NamespacedName) (*robotv1alpha1.MetricsCollector, error) {
	instance := &robotv1alpha1.MetricsCollector{}
	err := r.Get(ctx, meta, instance)
	if err != nil {
		return &robotv1alpha1.MetricsCollector{}, err
	}

	return instance, nil
}

func (r *MetricsCollectorReconciler) reconcileUpdateInstanceStatus(ctx context.Context, instance *robotv1alpha1.MetricsCollector) error {
	return retry.RetryOnConflict(retry.DefaultRetry, func() error {
		instanceLV := &robotv1alpha1.MetricsCollector{}
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

func (r *MetricsCollectorReconciler) reconcileGetTargetRobot(ctx context.Context, instance *robotv1alpha1.MetricsCollector) (*robotv1alpha1.Robot, error) {
	robot := &robotv1alpha1.Robot{}
	err := r.Get(ctx, types.NamespacedName{
		Namespace: instance.Namespace,
		Name:      label.GetTargetRobot(instance),
	}, robot)
	if err != nil {
		return nil, err
	}

	return robot, nil
}
