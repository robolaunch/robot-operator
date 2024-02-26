package v1alpha2

import (
	"github.com/robolaunch/robot-operator/internal"
	"k8s.io/apimachinery/pkg/types"
)

// ********************************
// ROS2Workload helpers
// ********************************

func (ros2Workload *ROS2Workload) GetDiscoveryServerMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Name:      ros2Workload.Name + internal.DISCOVERY_SERVER_POSTFIX,
		Namespace: ros2Workload.Namespace,
	}
}
