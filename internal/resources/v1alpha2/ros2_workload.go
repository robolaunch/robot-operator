package v1alpha2_resources

import (
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/types"

	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	robotv1alpha2 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha2"
)

func GetDiscoveryServer(ros2Workload *robotv1alpha2.ROS2Workload, dsNamespacedName *types.NamespacedName) *robotv1alpha1.DiscoveryServer {

	discoveryServer := robotv1alpha1.DiscoveryServer{
		ObjectMeta: metav1.ObjectMeta{
			Name:      dsNamespacedName.Name,
			Namespace: dsNamespacedName.Namespace,
			Labels:    ros2Workload.Labels,
		},
		Spec: ros2Workload.Spec.DiscoveryServerTemplate,
	}

	return &discoveryServer

}

func GetROS2Bridge(ros2Workload *robotv1alpha2.ROS2Workload, r2bNamespacedName *types.NamespacedName) *robotv1alpha2.ROS2Bridge {

	ros2Bridge := robotv1alpha2.ROS2Bridge{
		ObjectMeta: metav1.ObjectMeta{
			Name:      r2bNamespacedName.Name,
			Namespace: r2bNamespacedName.Namespace,
			Labels:    ros2Workload.Labels,
		},
		Spec: ros2Workload.Spec.DiscoveryServerTemplate,
	}

	return &discoveryServer

}
