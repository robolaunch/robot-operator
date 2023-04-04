package robot_vdi

import (
	"context"

	"github.com/robolaunch/robot-operator/internal/handle"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	corev1 "k8s.io/api/core/v1"
	networkingv1 "k8s.io/api/networking/v1"
	"k8s.io/apimachinery/pkg/api/errors"
)

func (r *RobotVDIReconciler) reconcileCheckPVC(ctx context.Context, instance *robotv1alpha1.RobotVDI) error {

	pvcQuery := &corev1.PersistentVolumeClaim{}
	err := r.Get(ctx, *instance.GetRobotVDIPVCMetadata(), pvcQuery)
	if err != nil {
		if errors.IsNotFound(err) {
			instance.Status.PVCStatus.Created = false
		} else {
			return err
		}
	} else {
		instance.Status.PVCStatus.Created = true
	}

	return nil
}

func (r *RobotVDIReconciler) reconcileCheckServices(ctx context.Context, instance *robotv1alpha1.RobotVDI) error {

	serviceTCPQuery := &corev1.Service{}
	err := r.Get(ctx, *instance.GetRobotVDIServiceTCPMetadata(), serviceTCPQuery)
	if err != nil {
		if errors.IsNotFound(err) {
			instance.Status.ServiceTCPStatus.Created = false
		} else {
			return err
		}
	} else {
		instance.Status.ServiceTCPStatus.Created = true
	}

	serviceUDPQuery := &corev1.Service{}
	err = r.Get(ctx, *instance.GetRobotVDIServiceUDPMetadata(), serviceUDPQuery)
	if err != nil {
		if errors.IsNotFound(err) {
			instance.Status.ServiceUDPStatus.Created = false
		} else {
			return err
		}
	} else {
		instance.Status.ServiceUDPStatus.Created = true
	}

	return nil
}

func (r *RobotVDIReconciler) reconcileCheckPod(ctx context.Context, instance *robotv1alpha1.RobotVDI) error {

	vdiPodQuery := &corev1.Pod{}
	err := r.Get(ctx, *instance.GetRobotVDIPodMetadata(), vdiPodQuery)
	if err != nil {
		if errors.IsNotFound(err) {
			instance.Status.PodStatus = robotv1alpha1.OwnedPodStatus{}
		} else {
			return err
		}
	} else {

		err = handle.HandlePod(ctx, r, *vdiPodQuery)
		if err != nil {
			return err
		}

		instance.Status.PodStatus.Resource.Created = true
		instance.Status.PodStatus.Resource.Phase = string(vdiPodQuery.Status.Phase)
		instance.Status.PodStatus.IP = vdiPodQuery.Status.PodIP
	}

	return nil
}

func (r *RobotVDIReconciler) reconcileCheckIngress(ctx context.Context, instance *robotv1alpha1.RobotVDI) error {

	if instance.Spec.Ingress {
		ingressQuery := &networkingv1.Ingress{}
		err := r.Get(ctx, *instance.GetRobotVDIIngressMetadata(), ingressQuery)
		if err != nil {
			if errors.IsNotFound(err) {
				instance.Status.IngressStatus.Created = false
			} else {
				return err
			}
		} else {
			instance.Status.IngressStatus.Created = true
		}
	}

	return nil
}
