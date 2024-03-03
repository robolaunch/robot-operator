package ros2_workload

import (
	"context"
	"reflect"

	"github.com/robolaunch/robot-operator/internal/reference"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	robotv1alpha2 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha2"
	corev1 "k8s.io/api/core/v1"
	"k8s.io/apimachinery/pkg/api/errors"
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
			discoveryServerQuery.Spec = instance.Spec.DiscoveryServerTemplate
			err = r.Update(ctx, discoveryServerQuery)
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
			ros2BridgeQuery.Spec = instance.Spec.ROS2BridgeTemplate
			err = r.Update(ctx, ros2BridgeQuery)
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
