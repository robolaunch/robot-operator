package v1alpha1

import (
	"errors"

	"github.com/robolaunch/robot-operator/internal"
	"k8s.io/apimachinery/pkg/types"
)

// ********************************
// Robot helpers
// ********************************

func (robot *Robot) GetPVCVarMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Name:      robot.Name + internal.PVC_VAR_POSTFIX,
		Namespace: robot.Namespace,
	}
}

func (robot *Robot) GetPVCOptMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Name:      robot.Name + internal.PVC_OPT_POSTFIX,
		Namespace: robot.Namespace,
	}
}

func (robot *Robot) GetPVCUsrMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Name:      robot.Name + internal.PVC_USR_POSTFIX,
		Namespace: robot.Namespace,
	}
}

func (robot *Robot) GetPVCEtcMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Name:      robot.Name + internal.PVC_ETC_POSTFIX,
		Namespace: robot.Namespace,
	}
}

func (robot *Robot) GetPVCWorkspaceMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Name:      robot.Name + internal.PVC_WORKSPACE_POSTFIX,
		Namespace: robot.Namespace,
	}
}

func (robot *Robot) GetDiscoveryServerMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Name:      robot.Name + internal.DISCOVERY_SERVER_POSTFIX,
		Namespace: robot.Namespace,
	}
}

func (robot *Robot) GetLoaderJobMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Name:      robot.Name + internal.JOB_LOADER_POSTFIX,
		Namespace: robot.Namespace,
	}
}

func (robot *Robot) GetROSBridgeMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Name:      robot.Name + internal.ROS_BRIDGE_POSTFIX,
		Namespace: robot.Namespace,
	}
}

func (robot *Robot) GetRobotDevSuiteMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Name:      robot.Name + internal.ROBOT_DEV_SUITE_POSTFIX,
		Namespace: robot.Namespace,
	}
}

func (robot *Robot) GetWorkspaceManagerMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Name:      robot.Name + internal.WORKSPACE_MANAGER_POSTFIX,
		Namespace: robot.Namespace,
	}
}

func (robot *Robot) GetWorkspaceByName(name string) (Workspace, error) {

	for _, ws := range robot.Spec.WorkspaceManagerTemplate.Workspaces {
		if ws.Name == name {
			return ws, nil
		}
	}

	return Workspace{}, errors.New("workspace not found")
}

// ********************************
// DiscoveryServer helpers
// *******************************

func (discoveryServer *DiscoveryServer) GetDiscoveryServerPodMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Name:      discoveryServer.Name,
		Namespace: discoveryServer.Namespace,
	}
}

func (discoveryServer *DiscoveryServer) GetDiscoveryServerServiceMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Name:      discoveryServer.Name + "-" + discoveryServer.Spec.Subdomain,
		Namespace: discoveryServer.Namespace,
	}
}

func (discoveryServer *DiscoveryServer) GetDiscoveryServerConfigMapMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Name:      discoveryServer.Name,
		Namespace: discoveryServer.Namespace,
	}
}

// ********************************
// ROSBridge helpers
// *******************************

func (rosbridge *ROSBridge) GetBridgePodMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Name:      rosbridge.Name,
		Namespace: rosbridge.Namespace,
	}
}

func (rosbridge *ROSBridge) GetBridgeServiceMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Name:      rosbridge.Name,
		Namespace: rosbridge.Namespace,
	}
}

func (rosbridge *ROSBridge) GetBridgeIngressMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Name:      rosbridge.Name,
		Namespace: rosbridge.Namespace,
	}
}

func (rosbridge *ROSBridge) GetOwnerMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Name:      rosbridge.OwnerReferences[0].Name,
		Namespace: rosbridge.Namespace,
	}
}
