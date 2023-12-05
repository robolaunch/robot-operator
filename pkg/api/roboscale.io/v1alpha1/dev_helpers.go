package v1alpha1

import (
	"github.com/robolaunch/robot-operator/internal"
	"github.com/robolaunch/robot-operator/internal/label"
	"k8s.io/apimachinery/pkg/types"
)

// ********************************
// RobotDevSuite helpers
// ********************************

func (robotDevSuite *RobotDevSuite) GetRobotVDIMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Namespace: robotDevSuite.Namespace,
		Name:      robotDevSuite.Name + internal.ROBOT_VDI_POSTFIX,
	}
}

func (robotDevSuite *RobotDevSuite) GetRobotIDEMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Namespace: robotDevSuite.Namespace,
		Name:      robotDevSuite.Name + internal.ROBOT_IDE_POSTFIX,
	}
}

func (robotDevSuite *RobotDevSuite) GetRemoteIDERelayServerMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Namespace: robotDevSuite.Namespace,
		Name:      robotDevSuite.Name + "-" + robotDevSuite.Spec.RemoteIDERelayServerTemplate.InstanceName + internal.REMOTE_IDE_RELAY_SERVER_POSTFIX,
	}
}

// ********************************
// RobotIDE helpers
// ********************************

func (robotide *RobotIDE) GetRobotIDEPodMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Namespace: robotide.Namespace,
		Name:      robotide.Name + internal.POD_IDE_POSTFIX,
	}
}

func (robotide *RobotIDE) GetRobotIDEServiceMetadata() *types.NamespacedName {
	instanceType := label.GetInstanceType(robotide)
	if instanceType == label.InstanceTypeCloudInstance {
		return &types.NamespacedName{
			Namespace: robotide.Namespace,
			Name:      robotide.Name + internal.SVC_IDE_POSTFIX,
		}
	} else {

		tenancy := label.GetTenancy(robotide)

		return &types.NamespacedName{
			Namespace: robotide.Namespace,
			Name:      robotide.Name + internal.SVC_IDE_POSTFIX + "-" + tenancy.PhysicalInstance,
		}
	}
}

func (robotide *RobotIDE) GetRobotIDEServiceExportMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Namespace: robotide.Namespace,
		Name:      robotide.GetRobotIDEServiceMetadata().Name,
	}

}

func (robotide *RobotIDE) GetRobotIDEIngressMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Namespace: robotide.Namespace,
		Name:      robotide.Name + internal.INGRESS_IDE_POSTFIX,
	}
}

func (robotide *RobotIDE) GetRobotIDECustomServiceMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Namespace: robotide.Namespace,
		Name:      robotide.Name + internal.CUSTOM_PORT_SVC_IDE_POSTFIX,
	}
}

func (robotide *RobotIDE) GetRobotIDECustomIngressMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Namespace: robotide.Namespace,
		Name:      robotide.Name + internal.CUSTOM_PORT_INGRESS_IDE_POSTFIX,
	}
}

func (robotide *RobotIDE) GetRobotIDEConfigMapMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Namespace: robotide.Namespace,
		Name:      robotide.Name + internal.CONFIGMAP_IDE_POSTFIX,
	}
}

// ********************************
// RobotVDI helpers
// ********************************

func (robotvdi *RobotVDI) GetRobotVDIPVCMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Namespace: robotvdi.Namespace,
		Name:      robotvdi.Name + internal.PVC_VDI_POSTFIX,
	}
}

func (robotvdi *RobotVDI) GetRobotVDIPodMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Namespace: robotvdi.Namespace,
		Name:      robotvdi.Name + internal.POD_VDI_POSTFIX,
	}
}

func (robotvdi *RobotVDI) GetRobotVDIServiceTCPMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Namespace: robotvdi.Namespace,
		Name:      robotvdi.Name + internal.SVC_TCP_VDI_POSTFIX,
	}
}

func (robotvdi *RobotVDI) GetRobotVDIServiceUDPMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Namespace: robotvdi.Namespace,
		Name:      robotvdi.Name + internal.SVC_UDP_VDI_POSTFIX,
	}
}

func (robotvdi *RobotVDI) GetRobotVDIIngressMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Namespace: robotvdi.Namespace,
		Name:      robotvdi.Name + internal.INGRESS_VDI_POSTFIX,
	}
}

func (robotvdi *RobotVDI) GetRobotVDICustomServiceMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Namespace: robotvdi.Namespace,
		Name:      robotvdi.Name + internal.CUSTOM_PORT_SVC_IDE_POSTFIX,
	}
}

func (robotvdi *RobotVDI) GetRobotVDICustomIngressMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Namespace: robotvdi.Namespace,
		Name:      robotvdi.Name + internal.CUSTOM_PORT_INGRESS_IDE_POSTFIX,
	}
}
