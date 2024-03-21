package v1alpha2

import (
	"github.com/robolaunch/robot-operator/internal/label"
	"github.com/robolaunch/robot-operator/internal/platform"
	appsv1 "k8s.io/api/apps/v1"
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

type ROS2BridgeInstanceStatus struct {
	// Generic status for any owned resource.
	Resource OwnedResourceStatus `json:"resource,omitempty"`
	// Status of the ROS2Bridge instance.
	Status ROS2BridgeStatus `json:"status,omitempty"`
	// Address of the robot service that can be reached from outside.
	Connection string `json:"connection,omitempty"`
}

type OwnedPVCStatus struct {
	// Generic status for any owned resource.
	Resource OwnedResourceStatus `json:"resource,omitempty"`
	// Status of the ROS2Bridge instance.
	Status corev1.PersistentVolumeClaimStatus `json:"status,omitempty"`
}

type OwnedStatefulSetStatus struct {
	// Generic status for any owned resource.
	Resource OwnedResourceStatus `json:"resource,omitempty"`
	// Status of the StatefulSet.
	Status appsv1.StatefulSetStatus `json:"status,omitempty"`
	// Container statuses.
	ContainerStatuses []corev1.ContainerStatus `json:"containerStatuses,omitempty"`
}

type OwnedDeploymentStatus struct {
	// Generic status for any owned resource.
	Resource OwnedResourceStatus `json:"resource,omitempty"`
	// Status of the Deployment.
	Status appsv1.DeploymentStatus `json:"status,omitempty"`
	// Container statuses.
	ContainerStatuses []corev1.ContainerStatus `json:"containerStatuses,omitempty"`
}

type OwnedServiceStatus struct {
	// Generic status for any owned resource.
	Resource OwnedResourceStatus `json:"resource,omitempty"`
	// Connection URL.
	URLs map[string]string `json:"urls,omitempty"`
}

func GetServiceDNS(obj metav1.Object, prefix, postfix string) string {
	tenancy := label.GetTenancy(obj)
	platformMeta := label.GetPlatformMeta(obj)
	connectionStr := tenancy.Team + "." + platformMeta.Domain + GetServicePath(obj, postfix)

	if prefix != "" {
		connectionStr = prefix + connectionStr
	}

	return connectionStr
}

func GetServiceDNSWithNodePort(obj metav1.Object, prefix, port string) string {
	tenancy := label.GetTenancy(obj)
	platformMeta := label.GetPlatformMeta(obj)
	connectionStr := tenancy.Team + "." + platformMeta.Domain + ":" + port

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

func validateVersion(obj metav1.Object, app string, appVersion string) error {
	platformMeta := label.GetPlatformMeta(obj)

	_, err := platform.GetToolsImage(obj, platformMeta.Version, app, appVersion)
	if err != nil {
		return err
	}

	return nil
}
