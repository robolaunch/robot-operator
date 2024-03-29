package v1alpha2

import (
	"strconv"

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

func (ros2Workload *ROS2Workload) GetROS2BridgeMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Name:      ros2Workload.Name + internal.ROS_2_BRIDGE_POSTFIX,
		Namespace: ros2Workload.Namespace,
	}
}

func (ros2Workload *ROS2Workload) GetPersistentVolumeClaimMetadata(key int) *types.NamespacedName {
	return &types.NamespacedName{
		Name:      ros2Workload.Name + internal.PVC_POSTFIX + "-" + strconv.Itoa(key),
		Namespace: ros2Workload.Namespace,
	}
}

func (ros2Workload *ROS2Workload) GetStatefulSetMetadata(key int) *types.NamespacedName {
	return &types.NamespacedName{
		Name:      ros2Workload.Name + internal.STATEFULSET_POSTFIX + "-" + strconv.Itoa(key),
		Namespace: ros2Workload.Namespace,
	}
}
