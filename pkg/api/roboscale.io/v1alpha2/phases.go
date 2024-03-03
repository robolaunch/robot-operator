package v1alpha2

type ROS2WorkloadPhase string

const (
	ROS2WorkloadPhaseCreatingDiscoveryServer ROS2WorkloadPhase = "CreatingDiscoveryServer"
	ROS2WorkloadPhaseCreatingROS2Bridge      ROS2WorkloadPhase = "CreatingROS2Bridge"
	ROS2WorkloadPhaseCreatingPVCs            ROS2WorkloadPhase = "CreatingPVCs"
	ROS2WorkloadPhaseReady                   ROS2WorkloadPhase = "Ready"
)

type ROS2BridgePhase string

const (
	ROS2BridgePhaseCreatingService ROS2BridgePhase = "CreatingService"
	ROS2BridgePhaseCreatingPod     ROS2BridgePhase = "CreatingPod"
	ROS2BridgePhaseCreatingIngress ROS2BridgePhase = "CreatingIngress"
	ROS2BridgePhaseReady           ROS2BridgePhase = "Ready"
	ROS2BridgePhaseDeletingPod     ROS2BridgePhase = "DeletingPod"
	ROS2BridgePhaseDeletingService ROS2BridgePhase = "DeletingService"
)
