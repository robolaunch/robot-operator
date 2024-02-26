package v1alpha2

type ROS2WorkloadPhase string

const (
	ROS2WorkloadPhaseCreatingDiscoveryServer ROS2WorkloadPhase = "CreatingDiscoveryServer"
	ROS2WorkloadPhaseReady                   ROS2WorkloadPhase = "Ready"
)
