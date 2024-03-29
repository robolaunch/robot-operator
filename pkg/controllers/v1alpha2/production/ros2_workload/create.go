package ros2_workload

import (
	"context"

	v1alpha2_resources "github.com/robolaunch/robot-operator/internal/resources/v1alpha2"
	"k8s.io/apimachinery/pkg/api/errors"
	ctrl "sigs.k8s.io/controller-runtime"

	robotv1alpha2 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha2"
)

func (r *ROS2WorkloadReconciler) createDiscoveryServer(ctx context.Context, instance *robotv1alpha2.ROS2Workload) error {

	discoveryServer := v1alpha2_resources.GetROS2WorkloadDiscoveryServer(instance, instance.GetDiscoveryServerMetadata())

	err := ctrl.SetControllerReference(instance, discoveryServer, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, discoveryServer)
	if err != nil && errors.IsAlreadyExists(err) {
		return nil
	} else if err != nil {
		return err
	}

	logger.Info("STATUS: Discovery server " + discoveryServer.Name + " is created.")
	return nil
}

func (r *ROS2WorkloadReconciler) createROS2Bridge(ctx context.Context, instance *robotv1alpha2.ROS2Workload) error {

	ros2Bridge := v1alpha2_resources.GetROS2WorkloadROS2Bridge(instance, instance.GetROS2BridgeMetadata())

	err := ctrl.SetControllerReference(instance, ros2Bridge, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, ros2Bridge)
	if err != nil && errors.IsAlreadyExists(err) {
		return nil
	} else if err != nil {
		return err
	}

	logger.Info("STATUS: ROS 2 Bridge " + ros2Bridge.Name + " is created.")
	return nil
}

func (r *ROS2WorkloadReconciler) createPersistentVolumeClaim(ctx context.Context, instance *robotv1alpha2.ROS2Workload, key int) error {

	pvc := v1alpha2_resources.GetROS2WorkloadPersistentVolumeClaim(instance, instance.GetPersistentVolumeClaimMetadata(key), key)

	err := ctrl.SetControllerReference(instance, pvc, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, pvc)
	if err != nil && errors.IsAlreadyExists(err) {
		return nil
	} else if err != nil {
		return err
	}

	logger.Info("STATUS: PVC " + instance.GetPersistentVolumeClaimMetadata(key).Name + " is created.")
	return nil
}

func (r *ROS2WorkloadReconciler) createStatefulSet(ctx context.Context, instance *robotv1alpha2.ROS2Workload, key int) error {

	node, err := r.reconcileGetNode(ctx, instance)
	if err != nil {
		return err
	}

	statefulSet := v1alpha2_resources.GetROS2WorkloadStatefulSet(instance, instance.GetStatefulSetMetadata(key), key, *node)

	err = ctrl.SetControllerReference(instance, statefulSet, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, statefulSet)
	if err != nil && errors.IsAlreadyExists(err) {
		return nil
	} else if err != nil {
		return err
	}

	logger.Info("STATUS: StatefulSet " + instance.GetStatefulSetMetadata(key).Name + " is created.")
	return nil
}
