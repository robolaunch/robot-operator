package metrics

import (
	"context"

	robotErr "github.com/robolaunch/robot-operator/internal/error"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	corev1 "k8s.io/api/core/v1"
)

func (r *MetricsExporterReconciler) reconcileHandleRole(ctx context.Context, instance *robotv1alpha1.MetricsExporter) error {

	if !instance.Status.RoleStatus.Created {
		instance.Status.Phase = robotv1alpha1.MetricsExporterPhaseCreatingRole
		err := r.reconcileCreateRole(ctx, instance)
		if err != nil {
			return err
		}
		instance.Status.RoleStatus.Created = true

		return &robotErr.CreatingResourceError{
			ResourceKind:      "Role",
			ResourceName:      instance.GetMetricsExporterRoleMetadata().Name,
			ResourceNamespace: instance.GetMetricsExporterRoleMetadata().Namespace,
		}
	}

	return nil
}

func (r *MetricsExporterReconciler) reconcileHandleServiceAccount(ctx context.Context, instance *robotv1alpha1.MetricsExporter) error {

	if !instance.Status.ServiceAccountStatus.Created {

		instance.Status.Phase = robotv1alpha1.MetricsExporterPhaseCreatingServiceAccount
		err := r.reconcileCreateServiceAccount(ctx, instance)
		if err != nil {
			return err
		}
		instance.Status.ServiceAccountStatus.Created = true

		return &robotErr.CreatingResourceError{
			ResourceKind:      "ServiceAccount",
			ResourceName:      instance.GetMetricsExporterServiceAccountMetadata().Name,
			ResourceNamespace: instance.GetMetricsExporterServiceAccountMetadata().Namespace,
		}
	}

	return nil
}

func (r *MetricsExporterReconciler) reconcileHandleRoleBinding(ctx context.Context, instance *robotv1alpha1.MetricsExporter) error {

	if !instance.Status.RoleBindingStatus.Created {

		instance.Status.Phase = robotv1alpha1.MetricsExporterPhaseCreatingRoleBinding
		err := r.reconcileCreateRoleBinding(ctx, instance)
		if err != nil {
			return err
		}
		instance.Status.RoleBindingStatus.Created = true

		return &robotErr.CreatingResourceError{
			ResourceKind:      "RoleBinding",
			ResourceName:      instance.GetMetricsExporterRoleBindingMetadata().Name,
			ResourceNamespace: instance.GetMetricsExporterRoleBindingMetadata().Namespace,
		}
	}

	return nil
}

func (r *MetricsExporterReconciler) reconcileHandlePod(ctx context.Context, instance *robotv1alpha1.MetricsExporter) error {

	if !instance.Status.PodStatus.Created {

		instance.Status.Phase = robotv1alpha1.MetricsExporterPhaseCreatingPod
		err := r.reconcileCreatePod(ctx, instance)
		if err != nil {
			return err
		}
		instance.Status.PodStatus.Created = true

		return &robotErr.CreatingResourceError{
			ResourceKind:      "Pod",
			ResourceName:      instance.GetMetricsExporterPodMetadata().Name,
			ResourceNamespace: instance.GetMetricsExporterPodMetadata().Namespace,
		}
	}

	if instance.Status.PodStatus.Phase != string(corev1.PodRunning) {
		return &robotErr.WaitingForResourceError{
			ResourceKind:      "Pod",
			ResourceName:      instance.GetMetricsExporterPodMetadata().Name,
			ResourceNamespace: instance.GetMetricsExporterPodMetadata().Namespace,
		}
	}

	return nil
}
