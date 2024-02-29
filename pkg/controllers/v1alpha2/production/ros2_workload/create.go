package ros2_workload

import (
	"context"

	v1alpha2_resources "github.com/robolaunch/robot-operator/internal/resources/v1alpha2"
	"k8s.io/apimachinery/pkg/api/errors"
	"k8s.io/apimachinery/pkg/types"
	ctrl "sigs.k8s.io/controller-runtime"

	robotv1alpha2 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha2"
)

func (r *ROS2WorkloadReconciler) createDiscoveryServer(ctx context.Context, instance *robotv1alpha2.ROS2Workload, dsNamespacedName *types.NamespacedName) error {

	discoveryServer := v1alpha2_resources.GetDiscoveryServer(instance, dsNamespacedName)

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

func (r *ROS2WorkloadReconciler) createROS2Bridge(ctx context.Context, instance *robotv1alpha2.ROS2Workload, r2bNamespacedName *types.NamespacedName) error {

	ros2Bridge := v1alpha2_resources.GetROS2Bridge(instance, r2bNamespacedName)

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
