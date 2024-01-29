package metrics

import (
	"context"
	"errors"
	"strconv"
	"strings"

	"github.com/robolaunch/robot-operator/internal"
	robotErr "github.com/robolaunch/robot-operator/internal/error"
	"github.com/robolaunch/robot-operator/internal/label"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	corev1 "k8s.io/api/core/v1"
	"k8s.io/apimachinery/pkg/labels"
	"k8s.io/apimachinery/pkg/selection"
	"k8s.io/apimachinery/pkg/types"
	"k8s.io/client-go/util/retry"
	"sigs.k8s.io/controller-runtime/pkg/client"
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

func (r *MetricsExporterReconciler) reconcileCheckNode(ctx context.Context, instance *robotv1alpha1.MetricsExporter) (*corev1.Node, error) {

	tenancyMap := label.GetTenancyMap(instance)

	requirements := []labels.Requirement{}
	for k, v := range tenancyMap {
		newReq, err := labels.NewRequirement(k, selection.In, []string{v})
		if err != nil {
			return nil, err
		}
		requirements = append(requirements, *newReq)
	}

	nodeSelector := labels.NewSelector().Add(requirements...)

	nodes := &corev1.NodeList{}
	err := r.List(ctx, nodes, &client.ListOptions{
		LabelSelector: nodeSelector,
	})
	if err != nil {
		return nil, err
	}

	if len(nodes.Items) == 0 {
		return nil, &robotErr.NodeNotFoundError{
			ResourceKind:      instance.Kind,
			ResourceName:      instance.Name,
			ResourceNamespace: instance.Namespace,
		}
	} else if len(nodes.Items) > 1 {
		return nil, &robotErr.MultipleNodeFoundError{
			ResourceKind:      instance.Kind,
			ResourceName:      instance.Name,
			ResourceNamespace: instance.Namespace,
		}
	}

	return &nodes.Items[0], nil
}

func (r *MetricsExporterReconciler) reconcileCheckGPUCapacities(ctx context.Context, instance *robotv1alpha1.MetricsExporter) error {
	activeNode, err := r.reconcileCheckNode(ctx, instance)
	if err != nil {
		return err
	}

	gpuInstanceUsages := make(map[string]robotv1alpha1.GPUInstanceStatus)

	for name, capacity := range activeNode.Status.Capacity {
		if strings.Contains(name.String(), "nvidia.com") {
			gpuInstanceStatus := robotv1alpha1.GPUInstanceStatus{}
			gpuInstanceStatus.Capacity = capacity.String()
			gpuInstanceUsages[name.String()] = gpuInstanceStatus
		}
	}

	instance.Status.Usage.GPUInstanceUsage = gpuInstanceUsages

	return nil
}

func (r *MetricsExporterReconciler) reconcileCheckGPUConsumingPods(ctx context.Context, instance *robotv1alpha1.MetricsExporter) error {

	// Check pods' GPU resource allocations excluding Robot-owned pods

	requirements := []labels.Requirement{}
	newReq, err := labels.NewRequirement(internal.ORGANIZATION_LABEL_KEY, selection.DoesNotExist, []string{})
	if err != nil {
		return err
	}
	requirements = append(requirements, *newReq)

	podSelector := labels.NewSelector().Add(requirements...)

	podList := corev1.PodList{}

	err = r.List(ctx, &podList, &client.ListOptions{LabelSelector: podSelector})
	if err != nil {
		return nil
	}

	usages := map[string]int64{}

	for _, pod := range podList.Items {
		for _, container := range pod.Spec.Containers {
			for resourceType, vCore := range container.Resources.Limits {
				if _, isGPUResource := instance.Status.Usage.GPUInstanceUsage[resourceType.String()]; isGPUResource {
					if val, ok := usages[resourceType.String()]; ok {
						usages[resourceType.String()] = val + vCore.ToDec().Value()
					} else {
						usages[resourceType.String()] = vCore.ToDec().Value()
					}
				}
			}
		}
	}

	// Check Robots' GPU resource allocations

	robotList := robotv1alpha1.RobotList{}

	err = r.List(ctx, &robotList)
	if err != nil {
		return err
	}

	for _, robot := range robotList.Items {
		if robot.Spec.RobotDevSuiteTemplate.VDIEnabled {
			vdiSpec := robot.Spec.RobotDevSuiteTemplate.RobotVDITemplate
			if val, ok := usages[vdiSpec.Resources.GPUInstance]; ok {
				usages[vdiSpec.Resources.GPUInstance] = val + int64(vdiSpec.Resources.GPUCore)
			} else {
				usages[vdiSpec.Resources.GPUInstance] = int64(vdiSpec.Resources.GPUCore)
			}
		}

		if robot.Spec.RobotDevSuiteTemplate.IDEEnabled {
			ideSpec := robot.Spec.RobotDevSuiteTemplate.RobotIDETemplate
			if val, ok := usages[ideSpec.Resources.GPUInstance]; ok {
				usages[ideSpec.Resources.GPUInstance] = val + int64(ideSpec.Resources.GPUCore)
			} else {
				usages[ideSpec.Resources.GPUInstance] = int64(ideSpec.Resources.GPUCore)
			}
		}

		if robot.Spec.RobotDevSuiteTemplate.NotebookEnabled {
			notebookSpec := robot.Spec.RobotDevSuiteTemplate.NotebookTemplate
			if val, ok := usages[notebookSpec.Resources.GPUInstance]; ok {
				usages[notebookSpec.Resources.GPUInstance] = val + int64(notebookSpec.Resources.GPUCore)
			} else {
				usages[notebookSpec.Resources.GPUInstance] = int64(notebookSpec.Resources.GPUCore)
			}
		}
	}

	for resourceType, vCore := range usages {
		if gpuInstanceStatus, ok := instance.Status.Usage.GPUInstanceUsage[resourceType]; ok {
			gpuInstanceStatus.Allocated = strconv.Itoa(int(vCore))
			instance.Status.Usage.GPUInstanceUsage[resourceType] = gpuInstanceStatus
		}
	}

	return nil
}

func (r *MetricsExporterReconciler) reconcileCheckDCGMEndpoint(ctx context.Context, instance *robotv1alpha1.MetricsExporter) error {

	if instance.Spec.GPU.Track {
		tenancy := label.GetTenancy(instance)

		requirements := []labels.Requirement{}
		newReq, err := labels.NewRequirement("app.kubernetes.io/component", selection.In, []string{"dcgm-exporter"})
		if err != nil {
			return err
		}
		requirements = append(requirements, *newReq)

		dcgmLabelSelector := labels.NewSelector().Add(requirements...)

		services := &corev1.ServiceList{}
		err = r.List(ctx, services, &client.ListOptions{
			LabelSelector: dcgmLabelSelector,
		})
		if err != nil {
			return err
		}

		if len(services.Items) == 0 {
			return errors.New("no service found for dcgm exporter")
		} else if len(services.Items) > 1 {
			return errors.New("multiple services found for dcgm exporter")
		}

		svc := services.Items[0]
		portObj := corev1.ServicePort{}
		for _, svcPort := range svc.Spec.Ports {
			if svcPort.Name == "metrics" {
				portObj = svcPort
			}
		}

		if portObj.Port == 0 {
			return errors.New("cannot get dcgm metrics port")
		}

		if tenancy.CloudInstance == "" {
			return errors.New("cannot identify the cloud instance")
		}

		instance.Status.Usage.GPUDeviceStatuses.DCGMEndpoint = "http://" + svc.Name + "." + svc.Namespace + ".svc." + tenancy.CloudInstance + ".local:" + strconv.Itoa(int(portObj.Port)) + "/metrics"
	}

	return nil
}
