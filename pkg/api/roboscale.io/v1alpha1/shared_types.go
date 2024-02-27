package v1alpha1

import (
	"github.com/robolaunch/robot-operator/internal/label"
	corev1 "k8s.io/api/core/v1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
)

// Generic status for any owned resource.
type OwnedResourceStatus struct {
	// Shows if the owned resource is created.
	Created bool `json:"created"`
	// Reference to the owned resource.
	Reference corev1.ObjectReference `json:"reference,omitempty"`
	// Phase of the owned resource.
	Phase string `json:"phase,omitempty"`
}

type OwnedRobotServiceStatus struct {
	// Generic status for any owned resource.
	Resource OwnedResourceStatus `json:"resource,omitempty"`
	// Address of the robot service that can be reached from outside.
	Connections map[string]string `json:"connections,omitempty"`
}

type OwnedServiceStatus struct {
	// Generic status for any owned resource.
	Resource OwnedResourceStatus `json:"resource,omitempty"`
	// Connection URL.
	URLs map[string]string `json:"urls,omitempty"`
}

type OwnedPodStatus struct {
	// Generic status for any owned resource.
	Resource OwnedResourceStatus `json:"resource,omitempty"`
	// IP of the pod.
	IP string `json:"ip,omitempty"`
}

type DiscoveryServerInstanceStatus struct {
	// Generic status for any owned resource.
	Resource OwnedResourceStatus `json:"resource,omitempty"`
	// Status of the DiscoveryServer instance.
	Status DiscoveryServerStatus `json:"status,omitempty"`
}

type ROSBridgeInstanceStatus struct {
	// Generic status for any owned resource.
	Resource OwnedResourceStatus `json:"resource,omitempty"`
	// Status of the ROSBridge instance.
	Status ROSBridgeStatus `json:"status,omitempty"`
	// Address of the robot service that can be reached from outside.
	Connection string `json:"connection,omitempty"`
}

type RobotDevSuiteInstanceStatus struct {
	// Generic status for any owned resource.
	Resource OwnedResourceStatus `json:"resource,omitempty"`
	// Status of the RobotDevSuite instance.
	Status RobotDevSuiteStatus `json:"status,omitempty"`
}

type WorkspaceManagerInstanceStatus struct {
	// Generic status for any owned resource.
	Resource OwnedResourceStatus `json:"resource,omitempty"`
	// Status of the WorkspaceManager instance.
	Status WorkspaceManagerStatus `json:"status,omitempty"`
}

type StepStatus struct {
	// Generic status for any owned resource.
	Resource OwnedResourceStatus `json:"resource,omitempty"`
	// Status of the step.
	Step Step `json:"step,omitempty"`
}

// DEPRECATE
func GetRobotServiceDNSWithNodePort(robot Robot, prefix, port string) string {
	tenancy := label.GetTenancy(&robot)
	connectionStr := tenancy.Team + "." + robot.Spec.RootDNSConfig.Host + ":" + port

	if prefix != "" {
		connectionStr = prefix + connectionStr
	}

	return connectionStr
}

// DEPRECATE
func GetRobotServiceDNS(robot Robot, prefix, postfix string) string {
	tenancy := label.GetTenancy(&robot)
	connectionStr := tenancy.Team + "." + robot.Spec.RootDNSConfig.Host + GetRobotServicePath(robot, postfix)

	if prefix != "" {
		connectionStr = prefix + connectionStr
	}

	return connectionStr
}

// DEPRECATE
func GetRelayServerServiceDNS(rs RelayServer, prefix, postfix string) string {
	tenancy := label.GetTenancy(&rs)
	connectionStr := tenancy.Team + "." + rs.Spec.RootDNSConfig.Host + GetRelayServerServicePath(rs, postfix)

	if prefix != "" {
		connectionStr = prefix + connectionStr
	}

	return connectionStr
}

// DEPRECATE
func GetRobotServicePath(robot Robot, postfix string) string {
	tenancy := label.GetTenancy(&robot)
	connectionStr := "/" + tenancy.Region +
		"/" + tenancy.CloudInstanceAlias +
		"/" + robot.Namespace +
		"/" + robot.Name

	if postfix != "" {
		connectionStr = connectionStr + postfix
	}

	return connectionStr
}

// DEPRECATE
func GetRelayServerServicePath(rs RelayServer, postfix string) string {
	tenancy := label.GetTenancy(&rs)
	connectionStr := "/" + tenancy.Region +
		"/" + tenancy.CloudInstance +
		"/" + rs.Namespace +
		"/" + rs.Name

	if postfix != "" {
		connectionStr = connectionStr + postfix
	}

	return connectionStr
}

func GetServiceDNS(obj metav1.Object, prefix, postfix string) string {
	tenancy := label.GetTenancy(obj)
	connectionStr := tenancy.Team + "." + tenancy.Domain + GetServicePath(obj, postfix)

	if prefix != "" {
		connectionStr = prefix + connectionStr
	}

	return connectionStr
}

func GetServicePath(obj metav1.Object, postfix string) string {
	tenancy := label.GetTenancy(obj)
	connectionStr := "/" + tenancy.Region +
		"/" + tenancy.CloudInstanceAlias +
		"/" + obj.GetNamespace() +
		"/" + obj.GetName()

	if postfix != "" {
		connectionStr = connectionStr + postfix
	}

	return connectionStr
}

func GetServiceDNSWithNodePort(obj metav1.Object, prefix, port string) string {
	tenancy := label.GetTenancy(obj)
	connectionStr := tenancy.Team + "." + tenancy.Domain + ":" + port

	if prefix != "" {
		connectionStr = prefix + connectionStr
	}

	return connectionStr
}
