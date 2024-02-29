package v1alpha2

import "k8s.io/apimachinery/pkg/types"

// ********************************
// ROS2Bridge helpers
// *******************************

func (ros2bridge *ROS2Bridge) GetROS2BridgePodMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Name:      ros2bridge.Name,
		Namespace: ros2bridge.Namespace,
	}
}

func (ros2bridge *ROS2Bridge) GetROS2BridgeServiceMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Name:      ros2bridge.Name,
		Namespace: ros2bridge.Namespace,
	}
}

func (ros2bridge *ROS2Bridge) GetROS2BridgeIngressMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Name:      ros2bridge.Name,
		Namespace: ros2bridge.Namespace,
	}
}
