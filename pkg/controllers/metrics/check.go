package metrics

import (
	"context"

	"github.com/robolaunch/robot-operator/internal/handle"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	corev1 "k8s.io/api/core/v1"
	rbacv1 "k8s.io/api/rbac/v1"
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

func (r *MetricsExporterReconciler) reconcileCheckRole(ctx context.Context, instance *robotv1alpha1.MetricsExporter) error {

	roleQuery := &rbacv1.Role{}
	err := r.Get(ctx, *instance.GetMetricsExporterRoleMetadata(), roleQuery)
	if err != nil {
		if errors.IsNotFound(err) {
			instance.Status.RoleStatus = robotv1alpha1.MetricsExporterRoleStatus{}
		} else {
			return err
		}
	} else {
		instance.Status.RoleStatus.Created = true
	}

	return nil
}

func (r *MetricsExporterReconciler) reconcileCheckRoleBinding(ctx context.Context, instance *robotv1alpha1.MetricsExporter) error {

	roleBindingQuery := &rbacv1.RoleBinding{}
	err := r.Get(ctx, *instance.GetMetricsExporterRoleBindingMetadata(), roleBindingQuery)
	if err != nil {
		if errors.IsNotFound(err) {
			instance.Status.RoleBindingStatus = robotv1alpha1.MetricsExporterRoleBindingStatus{}
		} else {
			return err
		}
	} else {
		instance.Status.RoleBindingStatus.Created = true
	}

	return nil
}

func (r *MetricsExporterReconciler) reconcileCheckServiceAccount(ctx context.Context, instance *robotv1alpha1.MetricsExporter) error {

	serviceAccountQuery := &corev1.ServiceAccount{}
	err := r.Get(ctx, *instance.GetMetricsExporterServiceAccountMetadata(), serviceAccountQuery)
	if err != nil {
		if errors.IsNotFound(err) {
			instance.Status.ServiceAccountStatus = robotv1alpha1.MetricsExporterServiceAccountStatus{}
		} else {
			return err
		}
	} else {
		instance.Status.ServiceAccountStatus.Created = true
	}

	return nil
}
