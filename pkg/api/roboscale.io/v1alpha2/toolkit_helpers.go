package v1alpha2

import (
	"strconv"

	"github.com/robolaunch/robot-operator/internal"
	"k8s.io/apimachinery/pkg/types"
)

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

// ********************************
// CodeEditor helpers
// *******************************

func (codeEditor *CodeEditor) GetPersistentVolumeClaimMetadata(key int) *types.NamespacedName {
	return &types.NamespacedName{
		Name:      codeEditor.Name + internal.PVC_POSTFIX + "-" + strconv.Itoa(key),
		Namespace: codeEditor.Namespace,
	}
}

func (codeEditor *CodeEditor) GetDeploymentMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Name:      codeEditor.Name + internal.DEPLOYMENT_POSTFIX,
		Namespace: codeEditor.Namespace,
	}
}

func (codeEditor *CodeEditor) GetServiceMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Name:      codeEditor.Name + internal.SERVICE_POSTFIX,
		Namespace: codeEditor.Namespace,
	}
}

func (codeEditor *CodeEditor) GetIngressMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Name:      codeEditor.Name + internal.INGRESS_POSTFIX,
		Namespace: codeEditor.Namespace,
	}
}

// ********************************
// EdgeProxy helpers
// *******************************

func (edgeProxy *EdgeProxy) GetDeploymentMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Name:      edgeProxy.Name + internal.DEPLOYMENT_POSTFIX,
		Namespace: edgeProxy.Namespace,
	}
}

func (edgeProxy *EdgeProxy) GetServiceMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Name:      edgeProxy.Name + internal.SERVICE_POSTFIX,
		Namespace: edgeProxy.Namespace,
	}
}

func (edgeProxy *EdgeProxy) GetIngressMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Name:      edgeProxy.Name + internal.INGRESS_POSTFIX,
		Namespace: edgeProxy.Namespace,
	}
}
