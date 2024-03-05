package ros2_workload

import (
	"context"
	"reflect"

	"github.com/robolaunch/robot-operator/internal/reference"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	robotv1alpha2 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha2"
	appsv1 "k8s.io/api/apps/v1"
	corev1 "k8s.io/api/core/v1"
	"k8s.io/apimachinery/pkg/api/errors"
	"k8s.io/apimachinery/pkg/labels"
	"k8s.io/apimachinery/pkg/selection"
	"sigs.k8s.io/controller-runtime/pkg/client"
)

func (r *ROS2WorkloadReconciler) reconcileCheckDiscoveryServer(ctx context.Context, instance *robotv1alpha2.ROS2Workload) error {

	discoveryServerQuery := &robotv1alpha1.DiscoveryServer{}
	err := r.Get(ctx, *instance.GetDiscoveryServerMetadata(), discoveryServerQuery)
	if err != nil && errors.IsNotFound(err) {
		instance.Status.DiscoveryServerStatus = robotv1alpha1.DiscoveryServerInstanceStatus{}
	} else if err != nil {
		return err
	} else {

		if !reflect.DeepEqual(instance.Spec.DiscoveryServerTemplate, discoveryServerQuery.Spec) {
			err = r.updateDiscoveryServer(ctx, instance)
			if err != nil {
				return err
			}
		}

		instance.Status.DiscoveryServerStatus.Resource.Created = true
		reference.SetReference(&instance.Status.DiscoveryServerStatus.Resource.Reference, discoveryServerQuery.TypeMeta, discoveryServerQuery.ObjectMeta)
		instance.Status.DiscoveryServerStatus.Status = discoveryServerQuery.Status
	}

	return nil
}

func (r *ROS2WorkloadReconciler) reconcileCheckROS2Bridge(ctx context.Context, instance *robotv1alpha2.ROS2Workload) error {

	ros2BridgeQuery := &robotv1alpha2.ROS2Bridge{}
	err := r.Get(ctx, *instance.GetROS2BridgeMetadata(), ros2BridgeQuery)
	if err != nil && errors.IsNotFound(err) {
		instance.Status.ROS2BridgeStatus = robotv1alpha2.ROS2BridgeInstanceStatus{}
	} else if err != nil {
		return err
	} else {

		if !reflect.DeepEqual(instance.Spec.ROS2BridgeTemplate, ros2BridgeQuery.Spec) {
			err = r.updateROS2Bridge(ctx, instance)
			if err != nil {
				return err
			}
		}

		instance.Status.ROS2BridgeStatus.Resource.Created = true
		reference.SetReference(&instance.Status.ROS2BridgeStatus.Resource.Reference, ros2BridgeQuery.TypeMeta, ros2BridgeQuery.ObjectMeta)
		instance.Status.ROS2BridgeStatus.Status = ros2BridgeQuery.Status
	}

	return nil
}

func (r *ROS2WorkloadReconciler) reconcileCheckPVCs(ctx context.Context, instance *robotv1alpha2.ROS2Workload) error {

	for key, pvcStatus := range instance.Status.PVCStatuses {

		pvcQuery := &corev1.PersistentVolumeClaim{}
		err := r.Get(ctx, *instance.GetPersistentVolumeClaimMetadata(key), pvcQuery)
		if err != nil && errors.IsNotFound(err) {
			pvcStatus.Resource.Created = false
		} else if err != nil {
			return err
		} else {
			pvcStatus.Resource.Created = true
			reference.SetReference(&pvcStatus.Resource.Reference, pvcQuery.TypeMeta, pvcQuery.ObjectMeta)
			pvcStatus.Status = pvcQuery.Status
		}

		instance.Status.PVCStatuses[key] = pvcStatus

	}

	return nil
}

func (r *ROS2WorkloadReconciler) reconcileCheckStatefulSets(ctx context.Context, instance *robotv1alpha2.ROS2Workload) error {

	volumesReady := true

	for _, pvcStatus := range instance.Status.PVCStatuses {
		volumesReady = volumesReady && pvcStatus.Resource.Created
	}

	if instance.Status.DiscoveryServerStatus.Status.ConfigMapStatus.Created && volumesReady {
		for key, ssStatus := range instance.Status.StatefulSetStatuses {

			ssQuery := &appsv1.StatefulSet{}
			err := r.Get(ctx, *instance.GetStatefulSetMetadata(key), ssQuery)
			if err != nil && errors.IsNotFound(err) {
				ssStatus.Resource.Created = false
			} else if err != nil {
				return err
			} else {
				// check if there are any inconsistencies
				launchContainer := instance.Spec.Containers[key]
				actualContainer := ssQuery.Spec.Template.Spec.Containers[0]

				volumeMountsSynced := true
				for _, vmDesired := range launchContainer.Container.VolumeMounts {
					volumePresent := false
					for _, vmActual := range actualContainer.VolumeMounts {
						if reflect.DeepEqual(vmDesired, vmActual) {
							volumePresent = true
						}
					}
					volumeMountsSynced = volumeMountsSynced && volumePresent
				}

				// changes that needs resource updates
				if !reflect.DeepEqual(launchContainer.Replicas, ssQuery.Spec.Replicas) ||
					!reflect.DeepEqual(launchContainer.Container.Image, actualContainer.Image) ||
					!reflect.DeepEqual(launchContainer.Container.Command, actualContainer.Command) ||
					!reflect.DeepEqual(launchContainer.Container.Resources, actualContainer.Resources) ||
					!reflect.DeepEqual(launchContainer.Container.SecurityContext, actualContainer.SecurityContext) {

					err = r.updateStatefulSet(ctx, instance, key)
					if err != nil {
						return err
					}

					ssStatus.Resource.Created = false
					ssStatus.Status = appsv1.StatefulSetStatus{}
					ssStatus.ContainerStatuses = []corev1.ContainerStatus{}
					continue
				}

				// changes that needs resource recreation
				if !reflect.DeepEqual(launchContainer.Container.Name, actualContainer.Name) ||
					!reflect.DeepEqual(launchContainer.Container.SecurityContext, actualContainer.SecurityContext) ||
					!volumeMountsSynced {

					err = r.Delete(ctx, ssQuery)
					if err != nil {
						return err
					}

					ssStatus.Resource.Created = false
					ssStatus.Status = appsv1.StatefulSetStatus{}
					ssStatus.ContainerStatuses = []corev1.ContainerStatus{}
					continue
				}

				// update statefulset status
				ssStatus.Resource.Created = true
				reference.SetReference(&ssStatus.Resource.Reference, ssQuery.TypeMeta, ssQuery.ObjectMeta)
				ssStatus.Status = ssQuery.Status

				// update container statuses
				newReq, err := labels.NewRequirement("controller-revision-hash", selection.In, []string{ssQuery.Status.CurrentRevision})
				if err != nil {
					return err
				}
				podSelector := labels.NewSelector().Add([]labels.Requirement{*newReq}...)

				podList := corev1.PodList{}
				err = r.List(ctx, &podList, &client.ListOptions{
					LabelSelector: podSelector,
				})
				if err != nil && errors.IsNotFound(err) {
					ssStatus.ContainerStatuses = []corev1.ContainerStatus{}
				} else if err != nil {
					return err
				} else {
					containerStatuses := []corev1.ContainerStatus{}
					for _, pod := range podList.Items {
						containerStatuses = append(containerStatuses, pod.Status.ContainerStatuses...)
					}
					ssStatus.ContainerStatuses = containerStatuses
				}

			}

			instance.Status.StatefulSetStatuses[key] = ssStatus

		}
	}

	return nil
}
