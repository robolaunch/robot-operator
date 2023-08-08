package robot_ide

import (
	"context"

	robotErr "github.com/robolaunch/robot-operator/internal/error"
	"github.com/robolaunch/robot-operator/internal/label"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	corev1 "k8s.io/api/core/v1"
)

func (r *RobotIDEReconciler) reconcileHandleService(ctx context.Context, instance *robotv1alpha1.RobotIDE) error {

	if !instance.Status.ServiceStatus.Resource.Created {
		instance.Status.Phase = robotv1alpha1.RobotIDEPhaseCreatingService
		err := r.reconcileCreateService(ctx, instance)
		if err != nil {
			return err
		}
		instance.Status.ServiceStatus.Resource.Created = true

		return &robotErr.CreatingResourceError{
			ResourceKind:      "Service",
			ResourceName:      instance.GetRobotIDEServiceMetadata().Name,
			ResourceNamespace: instance.GetRobotIDEServiceMetadata().Namespace,
		}
	}

	return nil
}

func (r *RobotIDEReconciler) reconcileHandlePod(ctx context.Context, instance *robotv1alpha1.RobotIDE) error {

	if !instance.Status.PodStatus.Resource.Created {
		instance.Status.Phase = robotv1alpha1.RobotIDEPhaseCreatingPod
		err := r.reconcileCreatePod(ctx, instance)
		if err != nil {
			return err
		}
		instance.Status.PodStatus.Resource.Created = true

		return &robotErr.CreatingResourceError{
			ResourceKind:      "Pod",
			ResourceName:      instance.GetRobotIDEPodMetadata().Name,
			ResourceNamespace: instance.GetRobotIDEPodMetadata().Namespace,
		}
	}

	if instance.Status.PodStatus.Resource.Phase != string(corev1.PodRunning) {
		return &robotErr.WaitingForResourceError{
			ResourceKind:      "Pod",
			ResourceName:      instance.GetRobotIDEPodMetadata().Name,
			ResourceNamespace: instance.GetRobotIDEPodMetadata().Namespace,
		}
	}

	return nil
}

func (r *RobotIDEReconciler) reconcileHandleIngress(ctx context.Context, instance *robotv1alpha1.RobotIDE) error {

	if instance.Spec.Ingress {
		if !instance.Status.IngressStatus.Created {
			instance.Status.Phase = robotv1alpha1.RobotIDEPhaseCreatingIngress
			err := r.reconcileCreateIngress(ctx, instance)
			if err != nil {
				return err
			}
			instance.Status.IngressStatus.Created = true

			return &robotErr.CreatingResourceError{
				ResourceKind:      "Ingress",
				ResourceName:      instance.GetRobotIDEIngressMetadata().Name,
				ResourceNamespace: instance.GetRobotIDEIngressMetadata().Namespace,
			}
		}
	}

	return nil
}

func (r *RobotIDEReconciler) reconcileHandleServiceExport(ctx context.Context, instance *robotv1alpha1.RobotIDE) error {

	if label.GetInstanceType(instance) == label.InstanceTypePhysicalInstance {
		if !instance.Status.ServiceExportStatus.Created {
			err := r.reconcileCreateServiceExport(ctx, instance)
			if err != nil {
				return err
			}
			instance.Status.ServiceExportStatus.Created = true

			return &robotErr.CreatingResourceError{
				ResourceKind:      "ServiceExport",
				ResourceName:      instance.GetRobotIDEServiceExportMetadata().Name,
				ResourceNamespace: instance.GetRobotIDEServiceExportMetadata().Namespace,
			}
		}
	}

	return nil
}
