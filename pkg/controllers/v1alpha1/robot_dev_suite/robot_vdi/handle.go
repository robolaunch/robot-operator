package robot_vdi

import (
	"context"

	"github.com/robolaunch/robot-operator/internal"
	robotErr "github.com/robolaunch/robot-operator/internal/error"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	corev1 "k8s.io/api/core/v1"
)

func (r *RobotVDIReconciler) reconcileHandlePVC(ctx context.Context, instance *robotv1alpha1.RobotVDI) error {

	if !instance.Status.PVCStatus.Created {
		instance.Status.Phase = robotv1alpha1.RobotVDIPhaseCreatingPVC
		err := r.reconcileCreatePVC(ctx, instance)
		if err != nil {
			return err
		}
		instance.Status.PVCStatus.Created = true

		return &robotErr.CreatingResourceError{
			ResourceKind:      "PersistentVolumeClaim",
			ResourceName:      instance.GetRobotVDIPVCMetadata().Name,
			ResourceNamespace: instance.GetRobotVDIPVCMetadata().Namespace,
		}
	}

	return nil
}

func (r *RobotVDIReconciler) reconcileHandleServiceTCP(ctx context.Context, instance *robotv1alpha1.RobotVDI) error {

	if !instance.Status.ServiceTCPStatus.Resource.Created {
		instance.Status.Phase = robotv1alpha1.RobotVDIPhaseCreatingTCPService
		err := r.reconcileCreateServiceTCP(ctx, instance)
		if err != nil {
			return err
		}
		instance.Status.ServiceTCPStatus.Resource.Created = true

		return &robotErr.CreatingResourceError{
			ResourceKind:      "Service",
			ResourceName:      instance.GetRobotVDIServiceTCPMetadata().Name,
			ResourceNamespace: instance.GetRobotVDIServiceTCPMetadata().Namespace,
		}
	}

	return nil
}

func (r *RobotVDIReconciler) reconcileHandleServiceUDP(ctx context.Context, instance *robotv1alpha1.RobotVDI) error {

	if !instance.Status.ServiceUDPStatus.Created {
		instance.Status.Phase = robotv1alpha1.RobotVDIPhaseCreatingUDPService
		err := r.reconcileCreateServiceUDP(ctx, instance)
		if err != nil {
			return err
		}
		instance.Status.ServiceUDPStatus.Created = true

		return &robotErr.CreatingResourceError{
			ResourceKind:      "Service",
			ResourceName:      instance.GetRobotVDIServiceUDPMetadata().Name,
			ResourceNamespace: instance.GetRobotVDIServiceUDPMetadata().Namespace,
		}
	}

	return nil
}

func (r *RobotVDIReconciler) reconcileHandlePod(ctx context.Context, instance *robotv1alpha1.RobotVDI) error {

	if !instance.Status.PodStatus.Resource.Created {
		instance.Status.Phase = robotv1alpha1.RobotVDIPhaseCreatingPod
		err := r.reconcileCreatePod(ctx, instance)
		if err != nil {
			return err
		}
		instance.Status.PodStatus.Resource.Created = true

		return &robotErr.CreatingResourceError{
			ResourceKind:      "Pod",
			ResourceName:      instance.GetRobotVDIPodMetadata().Name,
			ResourceNamespace: instance.GetRobotVDIPodMetadata().Namespace,
		}
	}

	if instance.Status.PodStatus.Resource.Phase != string(corev1.PodRunning) {
		return &robotErr.WaitingForResourceError{
			ResourceKind:      "Pod",
			ResourceName:      instance.GetRobotVDIPodMetadata().Name,
			ResourceNamespace: instance.GetRobotVDIPodMetadata().Namespace,
		}
	}

	return nil
}

func (r *RobotVDIReconciler) reconcileHandleIngress(ctx context.Context, instance *robotv1alpha1.RobotVDI) error {

	if instance.Spec.Ingress {
		if !instance.Status.IngressStatus.Created {
			instance.Status.Phase = robotv1alpha1.RobotVDIPhaseCreatingIngress
			err := r.reconcileCreateIngress(ctx, instance)
			if err != nil {
				return err
			}
			instance.Status.IngressStatus.Created = true

			return &robotErr.CreatingResourceError{
				ResourceKind:      "Ingress",
				ResourceName:      instance.GetRobotVDIIngressMetadata().Name,
				ResourceNamespace: instance.GetRobotVDIIngressMetadata().Namespace,
			}
		}
	}

	return nil
}

func (r *RobotVDIReconciler) reconcileHandleCustomService(ctx context.Context, instance *robotv1alpha1.RobotVDI) error {

	robot, err := r.reconcileGetTargetRobot(ctx, instance)
	if err != nil {
		return err
	}

	if _, ok := robot.Spec.AdditionalConfigs[internal.VDI_CUSTOM_PORT_RANGE_KEY]; ok {
		if !instance.Status.CustomPortServiceStatus.Resource.Created {
			instance.Status.Phase = robotv1alpha1.RobotVDIPhaseCreatingCustomPortService
			err := r.reconcileCreateCustomService(ctx, instance)
			if err != nil {
				return err
			}
			instance.Status.CustomPortServiceStatus.Resource.Created = true

			return &robotErr.CreatingResourceError{
				ResourceKind:      "Service",
				ResourceName:      instance.GetRobotVDICustomServiceMetadata().Name,
				ResourceNamespace: instance.GetRobotVDICustomServiceMetadata().Namespace,
			}
		}
	}

	return nil
}

func (r *RobotVDIReconciler) reconcileHandleCustomIngress(ctx context.Context, instance *robotv1alpha1.RobotVDI) error {

	if instance.Spec.Ingress {

		robot, err := r.reconcileGetTargetRobot(ctx, instance)
		if err != nil {
			return err
		}

		if _, ok := robot.Spec.AdditionalConfigs[internal.VDI_CUSTOM_PORT_RANGE_KEY]; ok {
			if !instance.Status.CustomPortIngressStatus.Created {
				instance.Status.Phase = robotv1alpha1.RobotVDIPhaseCreatingCustomPortIngress
				err := r.reconcileCreateCustomIngress(ctx, instance)
				if err != nil {
					return err
				}
				instance.Status.CustomPortIngressStatus.Created = true

				return &robotErr.CreatingResourceError{
					ResourceKind:      "Ingress",
					ResourceName:      instance.GetRobotVDICustomIngressMetadata().Name,
					ResourceNamespace: instance.GetRobotVDICustomIngressMetadata().Namespace,
				}
			}
		}
	}

	return nil
}
