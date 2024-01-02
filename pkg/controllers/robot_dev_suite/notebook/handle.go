package notebook

import (
	"context"

	"github.com/robolaunch/robot-operator/internal"
	robotErr "github.com/robolaunch/robot-operator/internal/error"
	"github.com/robolaunch/robot-operator/internal/label"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	corev1 "k8s.io/api/core/v1"
)

func (r *NotebookReconciler) reconcileHandleService(ctx context.Context, instance *robotv1alpha1.Notebook) error {

	if !instance.Status.ServiceStatus.Resource.Created {
		instance.Status.Phase = robotv1alpha1.NotebookPhaseCreatingService
		err := r.reconcileCreateService(ctx, instance)
		if err != nil {
			return err
		}
		instance.Status.ServiceStatus.Resource.Created = true

		return &robotErr.CreatingResourceError{
			ResourceKind:      "Service",
			ResourceName:      instance.GetNotebookServiceMetadata().Name,
			ResourceNamespace: instance.GetNotebookServiceMetadata().Namespace,
		}
	}

	return nil
}

func (r *NotebookReconciler) reconcileHandlePod(ctx context.Context, instance *robotv1alpha1.Notebook) error {

	if !instance.Status.PodStatus.Resource.Created {
		instance.Status.Phase = robotv1alpha1.NotebookPhaseCreatingPod
		err := r.reconcileCreatePod(ctx, instance)
		if err != nil {
			return err
		}
		instance.Status.PodStatus.Resource.Created = true

		return &robotErr.CreatingResourceError{
			ResourceKind:      "Pod",
			ResourceName:      instance.GetNotebookPodMetadata().Name,
			ResourceNamespace: instance.GetNotebookPodMetadata().Namespace,
		}
	}

	if instance.Status.PodStatus.Resource.Phase != string(corev1.PodRunning) {
		return &robotErr.WaitingForResourceError{
			ResourceKind:      "Pod",
			ResourceName:      instance.GetNotebookPodMetadata().Name,
			ResourceNamespace: instance.GetNotebookPodMetadata().Namespace,
		}
	}

	return nil
}

func (r *NotebookReconciler) reconcileHandleIngress(ctx context.Context, instance *robotv1alpha1.Notebook) error {

	if instance.Spec.Ingress {
		if !instance.Status.IngressStatus.Created {
			instance.Status.Phase = robotv1alpha1.NotebookPhaseCreatingIngress
			err := r.reconcileCreateIngress(ctx, instance)
			if err != nil {
				return err
			}
			instance.Status.IngressStatus.Created = true

			return &robotErr.CreatingResourceError{
				ResourceKind:      "Ingress",
				ResourceName:      instance.GetNotebookIngressMetadata().Name,
				ResourceNamespace: instance.GetNotebookIngressMetadata().Namespace,
			}
		}
	}

	return nil
}

func (r *NotebookReconciler) reconcileHandleServiceExport(ctx context.Context, instance *robotv1alpha1.Notebook) error {

	if label.GetInstanceType(instance) == label.InstanceTypePhysicalInstance {
		if !instance.Status.ServiceExportStatus.Created {
			err := r.reconcileCreateServiceExport(ctx, instance)
			if err != nil {
				return err
			}
			instance.Status.ServiceExportStatus.Created = true

			return &robotErr.CreatingResourceError{
				ResourceKind:      "ServiceExport",
				ResourceName:      instance.GetNotebookServiceExportMetadata().Name,
				ResourceNamespace: instance.GetNotebookServiceExportMetadata().Namespace,
			}
		}
	}

	return nil
}

func (r *NotebookReconciler) reconcileHandleCustomService(ctx context.Context, instance *robotv1alpha1.Notebook) error {

	robot, err := r.reconcileGetTargetRobot(ctx, instance)
	if err != nil {
		return err
	}

	if config, ok := robot.Spec.AdditionalConfigs[internal.NOTEBOOK_CUSTOM_PORT_RANGE_KEY]; ok && config.ConfigType == robotv1alpha1.AdditionalConfigTypeOperator {
		if !instance.Status.CustomPortServiceStatus.Resource.Created {
			instance.Status.Phase = robotv1alpha1.NotebookPhaseCreatingCustomPortService
			err := r.reconcileCreateCustomService(ctx, instance)
			if err != nil {
				return err
			}
			instance.Status.CustomPortServiceStatus.Resource.Created = true

			return &robotErr.CreatingResourceError{
				ResourceKind:      "Service",
				ResourceName:      instance.GetNotebookCustomServiceMetadata().Name,
				ResourceNamespace: instance.GetNotebookCustomServiceMetadata().Namespace,
			}
		}
	}

	return nil
}

func (r *NotebookReconciler) reconcileHandleCustomIngress(ctx context.Context, instance *robotv1alpha1.Notebook) error {

	if instance.Spec.Ingress {

		robot, err := r.reconcileGetTargetRobot(ctx, instance)
		if err != nil {
			return err
		}

		if config, ok := robot.Spec.AdditionalConfigs[internal.NOTEBOOK_CUSTOM_PORT_RANGE_KEY]; ok && config.ConfigType == robotv1alpha1.AdditionalConfigTypeOperator {
			if !instance.Status.CustomPortIngressStatus.Created {
				instance.Status.Phase = robotv1alpha1.NotebookPhaseCreatingCustomPortIngress
				err := r.reconcileCreateCustomIngress(ctx, instance)
				if err != nil {
					return err
				}
				instance.Status.CustomPortIngressStatus.Created = true

				return &robotErr.CreatingResourceError{
					ResourceKind:      "Ingress",
					ResourceName:      instance.GetNotebookCustomIngressMetadata().Name,
					ResourceNamespace: instance.GetNotebookCustomIngressMetadata().Namespace,
				}
			}
		}
	}

	return nil
}

func (r *NotebookReconciler) reconcileHandleConfigMap(ctx context.Context, instance *robotv1alpha1.Notebook) error {

	if !instance.Status.ConfigMapStatus.Created {
		instance.Status.Phase = robotv1alpha1.NotebookPhaseCreatingConfigMap
		err := r.reconcileCreateConfigMap(ctx, instance)
		if err != nil {
			return err
		}
		instance.Status.ConfigMapStatus.Created = true

		return &robotErr.CreatingResourceError{
			ResourceKind:      "ConfigMap",
			ResourceName:      instance.GetNotebookConfigMapMetadata().Name,
			ResourceNamespace: instance.GetNotebookConfigMapMetadata().Namespace,
		}
	}

	return nil
}
