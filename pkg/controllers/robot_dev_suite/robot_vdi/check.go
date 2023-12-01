package robot_vdi

import (
	"context"
	"strconv"

	"github.com/robolaunch/robot-operator/internal"
	"github.com/robolaunch/robot-operator/internal/handle"
	"github.com/robolaunch/robot-operator/internal/reference"
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
			instance.Status.PVCStatus = robotv1alpha1.OwnedResourceStatus{}
		} else {
			return err
		}
	} else {
		instance.Status.PVCStatus.Created = true
		reference.SetReference(&instance.Status.PVCStatus.Reference, pvcQuery.TypeMeta, pvcQuery.ObjectMeta)
	}

	return nil
}

func (r *RobotVDIReconciler) reconcileCheckServices(ctx context.Context, instance *robotv1alpha1.RobotVDI) error {

	serviceTCPQuery := &corev1.Service{}
	err := r.Get(ctx, *instance.GetRobotVDIServiceTCPMetadata(), serviceTCPQuery)
	if err != nil {
		if errors.IsNotFound(err) {
			instance.Status.ServiceTCPStatus = robotv1alpha1.OwnedServiceStatus{}
		} else {
			return err
		}
	} else {
		robot, err := r.reconcileGetTargetRobot(ctx, instance)
		if err != nil {
			return err
		}

		instance.Status.ServiceTCPStatus.Resource.Created = true
		reference.SetReference(&instance.Status.ServiceTCPStatus.Resource.Reference, serviceTCPQuery.TypeMeta, serviceTCPQuery.ObjectMeta)
		if instance.Spec.Ingress {
			instance.Status.ServiceTCPStatus.URLs["vdi"] = robotv1alpha1.GetRobotServiceDNS(*robot, "https://", "/vdi/")
			instance.Status.ServiceTCPStatus.URLs["vdi-file-browser"] = robotv1alpha1.GetRobotServiceDNS(*robot, "https://", "/file-browser/vdi/")
		} else if instance.Spec.ServiceType == corev1.ServiceTypeNodePort {
			instance.Status.ServiceTCPStatus.URLs["vdi"] = robotv1alpha1.GetRobotServiceDNSWithNodePort(*robot, "http://", strconv.Itoa(int(serviceTCPQuery.Spec.Ports[0].NodePort)))
			instance.Status.ServiceTCPStatus.URLs["vdi-file-browser"] = robotv1alpha1.GetRobotServiceDNSWithNodePort(*robot, "http://", strconv.Itoa(int(serviceTCPQuery.Spec.Ports[0].NodePort)))
		}
	}

	serviceUDPQuery := &corev1.Service{}
	err = r.Get(ctx, *instance.GetRobotVDIServiceUDPMetadata(), serviceUDPQuery)
	if err != nil {
		if errors.IsNotFound(err) {
			instance.Status.ServiceUDPStatus = robotv1alpha1.OwnedResourceStatus{}
		} else {
			return err
		}
	} else {
		instance.Status.ServiceUDPStatus.Created = true
		reference.SetReference(&instance.Status.ServiceUDPStatus.Reference, serviceUDPQuery.TypeMeta, serviceUDPQuery.ObjectMeta)
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
		reference.SetReference(&instance.Status.PodStatus.Resource.Reference, vdiPodQuery.TypeMeta, vdiPodQuery.ObjectMeta)
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
				instance.Status.IngressStatus = robotv1alpha1.OwnedResourceStatus{}
			} else {
				return err
			}
		} else {
			instance.Status.IngressStatus.Created = true
			reference.SetReference(&instance.Status.IngressStatus.Reference, ingressQuery.TypeMeta, ingressQuery.ObjectMeta)
		}
	}

	return nil
}

func (r *RobotVDIReconciler) reconcileCheckCustomService(ctx context.Context, instance *robotv1alpha1.RobotVDI) error {

	robot, err := r.reconcileGetTargetRobot(ctx, instance)
	if err != nil {
		return err
	}

	if _, ok := robot.Spec.AdditionalConfigs[internal.VDI_CUSTOM_PORT_RANGE_KEY]; ok {
		customSvcQuery := &corev1.Service{}
		err := r.Get(ctx, *instance.GetRobotVDICustomServiceMetadata(), customSvcQuery)
		if err != nil {
			if errors.IsNotFound(err) {
				instance.Status.CustomPortServiceStatus = robotv1alpha1.OwnedServiceStatus{}
			} else {
				return err
			}
		} else {
			instance.Status.CustomPortServiceStatus.Resource.Created = true
			reference.SetReference(&instance.Status.CustomPortServiceStatus.Resource.Reference, customSvcQuery.TypeMeta, customSvcQuery.ObjectMeta)
		}
	}

	return nil
}

func (r *RobotVDIReconciler) reconcileCheckCustomIngress(ctx context.Context, instance *robotv1alpha1.RobotVDI) error {

	if instance.Spec.Ingress {
		customIngressQuery := &networkingv1.Ingress{}
		err := r.Get(ctx, *instance.GetRobotVDICustomIngressMetadata(), customIngressQuery)
		if err != nil {
			if errors.IsNotFound(err) {
				instance.Status.CustomPortIngressStatus = robotv1alpha1.OwnedResourceStatus{}
			} else {
				return err
			}
		} else {
			instance.Status.CustomPortIngressStatus.Created = true
			reference.SetReference(&instance.Status.CustomPortIngressStatus.Reference, customIngressQuery.TypeMeta, customIngressQuery.ObjectMeta)
		}
	}

	return nil
}
