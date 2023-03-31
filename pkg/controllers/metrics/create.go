package metrics

import (
	"context"

	"github.com/robolaunch/robot-operator/internal/resources"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	"k8s.io/apimachinery/pkg/api/errors"
	ctrl "sigs.k8s.io/controller-runtime"
)

func (r *MetricsExporterReconciler) reconcileCreatePod(ctx context.Context, instance *robotv1alpha1.MetricsExporter) error {

	activeNode, err := r.reconcileCheckNode(ctx, instance)
	if err != nil {
		return err
	}

	metricsPod := resources.GetMetricsExporterPod(instance, instance.GetMetricsExporterPodMetadata(), *activeNode)

	err = ctrl.SetControllerReference(instance, metricsPod, r.Scheme)
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

func (r *MetricsExporterReconciler) reconcileCreateRole(ctx context.Context, instance *robotv1alpha1.MetricsExporter) error {

	metricsRole := resources.GetMetricsExporterRole(instance, instance.GetMetricsExporterRoleMetadata())

	err := ctrl.SetControllerReference(instance, metricsRole, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, metricsRole)
	if err != nil && errors.IsAlreadyExists(err) {
		return nil
	} else if err != nil {
		return err
	}

	logger.Info("STATUS: Metrics role is created.")

	return nil
}

func (r *MetricsExporterReconciler) reconcileCreateRoleBinding(ctx context.Context, instance *robotv1alpha1.MetricsExporter) error {

	metricsRoleBinding := resources.GetMetricsExporterRoleBinding(instance, instance.GetMetricsExporterRoleBindingMetadata())

	err := ctrl.SetControllerReference(instance, metricsRoleBinding, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, metricsRoleBinding)
	if err != nil && errors.IsAlreadyExists(err) {
		return nil
	} else if err != nil {
		return err
	}

	logger.Info("STATUS: Metrics role binding is created.")

	return nil
}

func (r *MetricsExporterReconciler) reconcileCreateServiceAccount(ctx context.Context, instance *robotv1alpha1.MetricsExporter) error {

	metricsServiceAccount := resources.GetMetricsExporterServiceAccount(instance, instance.GetMetricsExporterServiceAccountMetadata())

	err := ctrl.SetControllerReference(instance, metricsServiceAccount, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, metricsServiceAccount)
	if err != nil && errors.IsAlreadyExists(err) {
		return nil
	} else if err != nil {
		return err
	}

	logger.Info("STATUS: Metrics service account is created.")

	return nil
}
